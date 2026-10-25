/*
 * =============================================================================
 * Autor:          Julian David Escobar Teatino
 * Fecha:          20 de octubre de 2026
 * Proyecto:       Consultoria tecnica IoT - Cooperativa hidroponica de la Sabana
 * Cultivo:        Lechuga crespa (Lactuca sativa) en losa de fibra de coco
 * Funcion:        Apertura proporcional de la valvula de riego por goteo segun
 *                 la humedad del sustrato, con barra luminosa para el operario.
 * Plataforma:     Arduino Uno, lenguaje C++, simulacion Tinkercad
 * Curso:          IoT 202087712 - UNAD - Fase 4 - Escenario 3
 * =============================================================================
 *
 * CALIBRACION (escala del simulador, no un porcentaje gravimetrico de campo)
 *   ADC_SECO     = 280  -> 0 %   sustrato seco / sensor en aire
 *   ADC_SATURADO = 900  -> 100 % sustrato a capacidad de contenedor
 *   Humedad % = mapa lineal entre esos dos puntos, limitado a 0..100.
 *
 *
 * CINCO ESTADOS (criterio agronomico resumido; el informe lo desarrolla)
 *   h >  78 %   valvula 0 deg,   0 LEDs  capacidad de contenedor: no regar
 *   h >  62 %   valvula 45 deg,  2 LEDs  caudal de mantenimiento
 *   h >  45 %   valvula 90 deg,  4 LEDs  caudal de referencia (ETc, Kc ~ 1)
 *   h >  30 %   valvula 135 deg, 6 LEDs  recuperacion del sustrato
 *   h <= 30 %   valvula 180 deg, 8 LEDs  estres hidrico, alerta en LCD
 *
 * El servo de 180 deg es la escala didactica pedida por la guia. Una valvula
 * esferica real es de un cuarto de vuelta (0 a 90 deg); el informe explica
 * como se trasladan estos cinco estados a ese actuador.
 *
 * BARRA
 * Tira NeoPixel de 8 LEDs, un solo cable de datos en el pin 6. Se encienden
 * de forma acumulativa: 0, 2, 4, 6 u 8. En estres hidrico los 8 quedan rojos 
 * para que la alerta se distinga
 * del caudal alto.
 *
 * INTEGRIDAD
 * Un salto que no se sostiene 350 ms no mueve la valvula: se conserva el
 * ultimo angulo valido. Asi una lectura falsa de "suelo seco" no abre el
 * riego de emergencia por un contacto sucio de un instante.
 * =============================================================================
 */

#include <Servo.h>
#include <LiquidCrystal.h>
#include <Adafruit_NeoPixel.h>

const int PIN_HUMEDAD = A0;
const int PIN_SERVO = 9;
const int PIN_NEOPIXEL = 6;
const int PIN_LATIDO = 13;
const int NUM_LEDS = 8;

// LCD en paralelo usando pines analogicos como salidas digitales.
// RS=12, E=11, D4=A1, D5=A2, D6=A3, D7=A4. A0 queda libre para el sensor.
LiquidCrystal lcd(12, 11, A1, A2, A3, A4);
Servo valvula;
Adafruit_NeoPixel barra(NUM_LEDS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

const int ADC_SECO = 280;
const int ADC_SATURADO = 900;

const int H_CIERRE = 78;       // Por encima: no regar
const int H_MANTENIMIENTO = 62;
const int H_REFERENCIA = 45;
const int H_RECUPERACION = 30; // En este valor y por debajo: emergencia

const int SALTO_ADC_MAX = 180;
const unsigned long T_ACEPTAR_SALTO_MS = 350;
const unsigned long T_LATIDO_MS = 500;
const unsigned long T_INFORME_MS = 400;

int ultimoAceptado = -1;
unsigned long tInicioSalto = 0;
unsigned long tLatido = 0;
unsigned long tInforme = 0;
bool latidoEncendido = false;
bool manteniendoLectura = false;

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

int lecturaConfiable(int muestra) {
  if (ultimoAceptado < 0) {
    ultimoAceptado = muestra;
    manteniendoLectura = false;
    return muestra;
  }
  if (abs(muestra - ultimoAceptado) > SALTO_ADC_MAX) {
    if (tInicioSalto == 0) {
      tInicioSalto = millis();
    }
    if (millis() - tInicioSalto < T_ACEPTAR_SALTO_MS) {
      manteniendoLectura = true;
      return ultimoAceptado;
    }
  }
  tInicioSalto = 0;
  manteniendoLectura = false;
  ultimoAceptado = muestra;
  return promedioMovil(muestra);
}

int humedadPorcentaje(int adc) {
  // map invierte la escala: ADC alto (seco) -> 0 %, ADC bajo (saturado) -> 100 %.
  int h = map(adc, ADC_SECO, ADC_SATURADO, 0, 100);
  if (h < 0) {
    h = 0;
  }
  if (h > 100) {
    h = 100;
  }
  return h;
}

void barraLeds(int cantidad, bool critico) {
  if (cantidad < 0) {
    cantidad = 0;
  }
  if (cantidad > NUM_LEDS) {
    cantidad = NUM_LEDS;
  }
  for (int i = 0; i < NUM_LEDS; i++) {
    if (i < cantidad) {
      // Rojo solo en estres. El resto de los pasos crece en verde.
      if (critico) {
        barra.setPixelColor(i, barra.Color(180, 0, 0));
      } else {
        barra.setPixelColor(i, barra.Color(0, 140, 0));
      }
    } else {
      barra.setPixelColor(i, 0);
    }
  }
  barra.show();
}

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

// Aplica un estado: angulo, LEDs y frase que el operario puede leer sin
// conocer el algoritmo. La barra crece de forma acumulativa.
void aplicarEstado(int humedad, int angulo, int leds, const char* caudal) {
  char fila1[17];
  char fila2[17];
  valvula.write(angulo);
  barraLeds(leds, leds == NUM_LEDS);
  snprintf(fila1, sizeof(fila1), "H%3d%% Valv %3d", humedad, angulo);
  snprintf(fila2, sizeof(fila2), "%-11s L%d", caudal, leds);
  escribirLcd(fila1, fila2);
}

void setup() {
  pinMode(PIN_LATIDO, OUTPUT);
  barra.begin();
  barra.show();
  Serial.begin(9600);
  lcd.begin(16, 2);
  valvula.attach(PIN_SERVO);
  valvula.write(0);
  escribirLcd("Riego lechuga", "Fibra de coco");
  Serial.println(F("ESCENARIO 3 - Riego proporcional - lechuga en coco"));
  Serial.println(F("h>78 cierre | >62 mant. | >45 ETc | >30 recup. | <=30 critico"));
  delay(1200);
}

void loop() {
  int adcCrudo = analogRead(PIN_HUMEDAD);
  int adc = lecturaConfiable(adcCrudo);
  int h = humedadPorcentaje(adc);
  int angulo = 0;

  // Rangos explicitos. Cada uno fija angulo y numero de LEDs, no una
  // proporcion continua, porque la valvula de este diseno tiene cinco
  // caudales operativos y el operario debe reconocerlos en la barra.
  if (h > H_CIERRE) {
    angulo = 0;
    aplicarEstado(h, angulo, 0, "Sin riego");
  } else if (h > H_MANTENIMIENTO) {
    angulo = 45;
    aplicarEstado(h, angulo, 2, "Caudal min");
  } else if (h > H_REFERENCIA) {
    angulo = 90;
    aplicarEstado(h, angulo, 4, "Caudal ETc");
  } else if (h > H_RECUPERACION) {
    angulo = 135;
    aplicarEstado(h, angulo, 6, "Caudal alto");
  } else {
    // Estres hidrico: apertura total y frase de alerta, no solo un numero.
    angulo = 180;
    aplicarEstado(h, angulo, 8, "CRITICO");
  }

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
    Serial.print(F(" h="));
    Serial.print(h);
    Serial.print(F(" servo="));
    Serial.print(angulo);
    Serial.print(F(" estable="));
    Serial.println(manteniendoLectura ? F("NO") : F("SI"));
  }
}
