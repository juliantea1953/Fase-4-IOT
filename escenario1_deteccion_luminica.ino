/*
 * =============================================================================
 * Autor:          Julian David Escobar Teatino
 * Fecha:          20 de octubre de 2026
 * Proyecto:       Consultoria tecnica IoT - Cooperativa hidroponica de la Sabana
 * Cultivo:        Lechuga crespa (Lactuca sativa) en invernadero, Cundinamarca
 * Funcion:        Alerta visual de luminosidad para el invernadero y la mesa
 *                 de clasificacion. Protege la labor del operario y evita
 *                 decidir el manejo del cultivo con luz insuficiente o con
 *                 exceso de radiacion.
 * Plataforma:     Arduino Uno, lenguaje C++, simulacion Tinkercad
 * Curso:          IoT 202087712 - UNAD - Fase 4 - Escenario 1
 * =============================================================================
 *
 * PRINCIPIO DE MEDIDA
 * La fotorresistencia (LDR) disminuye su resistencia cuando aumenta la luz.
 * Con el divisor 5 V - LDR - A0 - 10 kOhm - GND, mas luz produce mas tension
 * en A0 y por tanto un valor ADC mas alto (0 a 1023).
 *
 * UMBRALES (no son lux reales: el simulador no esta calibrado en lux).
 * Se interpretan asi, de forma conservadora, para esta cooperativa:
 *   ADC <  280  luz baja. Proxy de un puesto por debajo de lo que el RETILAP
 *               exige para una tarea visual de clasificacion, y de un
 *               fotoperiodo pobre para la lechuga. Alerta ROJA.
 *   ADC 280-720 operacion aceptable para clasificar y para el crecimiento.
 *               Alerta VERDE.
 *   ADC >  720  exceso de radiacion. En lechuga de invernadero, junto con
 *               alta temperatura, se asocia a riesgo de tipburn. Alerta AZUL.
 * Si al probar el circuito los colores quedan invertidos, el divisor quedo
 * al reves: intercambiar UMBRAL_BAJA y la logica, o voltear LDR y resistencia.
 * =============================================================================
 */

#include <LiquidCrystal.h>

// --- Pines ---
const int PIN_LDR = A0;       // Salida del divisor de tension de la fotorresistencia
const int PIN_ROJO = 9;       // Canal rojo del LED RGB (PWM)
const int PIN_VERDE = 10;     // Canal verde del LED RGB (PWM)
const int PIN_AZUL = 11;      // Canal azul del LED RGB (PWM)
const int PIN_LATIDO = 13;    // LED de vida del nodo (senal de que el lazo corre)

// LCD 16x2 en paralelo: RS, E, D4, D5, D6, D7
// Se evitan los pines PWM 9, 10 y 11, reservados al LED RGB.
LiquidCrystal lcd(12, 8, 7, 6, 5, 4);

// --- Umbrales agronomicos y de seguridad laboral sobre la escala del ADC ---
const int UMBRAL_BAJA = 280;
const int UMBRAL_ALTA = 720;

// Intensidad PWM moderada para no exigir la corriente maxima del pin.
const int BRILLO = 180;

// El salto brusco se ignora hasta que se sostiene. En planta, un destello
// (puerta, linterna) no debe cambiar la alerta. 350 ms no se percibe como
// retardo y si filtra transitorios.
const int SALTO_ADC_MAX = 160;
const unsigned long T_ACEPTAR_SALTO_MS = 350;
const unsigned long T_LATIDO_MS = 500;
const unsigned long T_INFORME_MS = 400;

int ultimoAceptado = -1;
unsigned long tInicioSalto = 0;
unsigned long tLatido = 0;
unsigned long tInforme = 0;
bool latidoEncendido = false;

// Promedio movil corto: suaviza el ruido del ADC sin esconder un cambio real.
const int VENTANA = 4;
int bufferAdc[VENTANA];
int indiceBuffer = 0;
long sumaBuffer = 0;
int muestrasEnBuffer = 0;

int promedioMovil(int muestra) {
  if (muestrasEnBuffer < VENTANA) {
    bufferAdc[muestrasEnBuffer] = muestra;
    sumaBuffer += muestra;
    muestrasEnBuffer++;
    return (int)(sumaBuffer / muestrasEnBuffer);
  }
  sumaBuffer -= bufferAdc[indiceBuffer];
  bufferAdc[indiceBuffer] = muestra;
  sumaBuffer += muestra;
  indiceBuffer = (indiceBuffer + 1) % VENTANA;
  return (int)(sumaBuffer / VENTANA);
}

// Devuelve la lectura que el control tiene derecho a usar.
// Un pico breve no mueve la alerta; un cambio que permanece, si.
int lecturaConfiable(int muestra) {
  if (ultimoAceptado < 0) {
    ultimoAceptado = muestra;
    return muestra;
  }
  if (abs(muestra - ultimoAceptado) > SALTO_ADC_MAX) {
    if (tInicioSalto == 0) {
      tInicioSalto = millis();
    }
    if (millis() - tInicioSalto < T_ACEPTAR_SALTO_MS) {
      return ultimoAceptado;
    }
  }
  tInicioSalto = 0;
  ultimoAceptado = muestra;
  return promedioMovil(muestra);
}

void pintarRgb(int rojo, int verde, int azul) {
  analogWrite(PIN_ROJO, rojo);
  analogWrite(PIN_VERDE, verde);
  analogWrite(PIN_AZUL, azul);
}

// El HD44780 del simulador no muestra tildes de forma fiable.
// El texto va en ASCII para que el operario lea una frase, no simbolos.
void escribirLcd(const char* linea1, const char* linea2) {
  char fila1[17];
  char fila2[17];
  snprintf(fila1, sizeof(fila1), "%-16s", linea1);
  snprintf(fila2, sizeof(fila2), "%-16s", linea2);
  lcd.setCursor(0, 0);
  lcd.print(fila1);
  lcd.setCursor(0, 1);
  lcd.print(fila2);
}

void setup() {
  pinMode(PIN_ROJO, OUTPUT);
  pinMode(PIN_VERDE, OUTPUT);
  pinMode(PIN_AZUL, OUTPUT);
  pinMode(PIN_LATIDO, OUTPUT);

  Serial.begin(9600);
  lcd.begin(16, 2);
  escribirLcd("Coop. Sabana", "Sensor de luz");

  Serial.println(F("ESCENARIO 1 - Deteccion luminica - lechuga de invernadero"));
  Serial.println(F("ADC < 280  ROJO  luz insuficiente para clasificar"));
  Serial.println(F("280 a 720  VERDE operacion aceptable"));
  Serial.println(F("ADC > 720  AZUL  exceso de radiacion"));
  delay(1200);
}

void loop() {
  int adcCrudo = analogRead(PIN_LDR);
  int adc = lecturaConfiable(adcCrudo);

  // Tres estados. Cada uno tiene un color, un texto y una razon distinta.
  if (adc < UMBRAL_BAJA) {
    pintarRgb(BRILLO, 0, 0);
    escribirLcd("LUZ BAJA", "Riesgo laboral");
  } else if (adc <= UMBRAL_ALTA) {
    pintarRgb(0, BRILLO, 0);
    escribirLcd("LUZ ACEPTABLE", "Operacion normal");
  } else {
    pintarRgb(0, 0, BRILLO);
    escribirLcd("EXCESO DE LUZ", "Riesgo cultivo");
  }

  // Senal de vida: si este LED deja de parpadear, el nodo esta detenido.
  // Es el mismo concepto de heartbeat usado en la arquitectura de red.
  if (millis() - tLatido >= T_LATIDO_MS) {
    tLatido = millis();
    latidoEncendido = !latidoEncendido;
    digitalWrite(PIN_LATIDO, latidoEncendido ? HIGH : LOW);
  }

  if (millis() - tInforme >= T_INFORME_MS) {
    tInforme = millis();
    Serial.print(F("ADC_crudo="));
    Serial.print(adcCrudo);
    Serial.print(F(" ADC_usado="));
    Serial.print(adc);
    Serial.print(F(" estado="));
    if (adc < UMBRAL_BAJA) {
      Serial.println(F("LUZ_BAJA color=ROJO"));
    } else if (adc <= UMBRAL_ALTA) {
      Serial.println(F("ACEPTABLE color=VERDE"));
    } else {
      Serial.println(F("EXCESO color=AZUL"));
    }
  }
}
