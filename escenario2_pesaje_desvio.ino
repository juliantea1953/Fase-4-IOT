/*
 * =============================================================================
 * Autor:          Julian David Escobar Teatino
 * Fecha:          20 de octubre de 2026
 * Proyecto:       Consultoria tecnica IoT - Cooperativa hidroponica de la Sabana
 * Producto:       Bandeja de lechuga crespa hidroponica, peso objetivo 500 g
 * Funcion:        Pesaje proporcional de la bandeja activa y desvio automatico
 *                 de la tolva hacia la siguiente bandeja, sin detener el flujo.
 * Plataforma:     Arduino Uno, lenguaje C++, simulacion Tinkercad
 * Curso:          IoT 202087712 - UNAD - Fase 4 - Escenario 2
 * =============================================================================
 *
 * CALIBRACION DEL SENSOR DE FUERZA (FSR) EN EL SIMULADOR
 * Divisor: 5 V - FSR - A0 - 10 kOhm - GND.
 * ADC_CERO es la lectura con la bandeja vacia (ruido, no peso).
 * ADC_OBJETIVO es la lectura que en este montaje equivale a 500 g.
 * El FSR no es una bascula legal: en planta se cambia por celda de carga
 * e HX711. La logica de estados no cambia; solo cambia la fuente del entero
 * "peso en gramos".
 *
 * ESTADOS
 *   LLENANDO         servo en flujo directo (0 grados). La barra Neopixel
 *                    enciende LEDs en proporcion al peso: 25 % -> 4 LEDs,
 *                    50 % -> 8, 75 % -> 12, 100 % -> 16.
 *   CONFIRMANDO      el peso filtrado llevo al menos 300 ms en el objetivo.
 *                    El servo pasa a desvio (90 grados), los 16 LEDs confirman
 *                    el llenado y el LCD avisa al operario.
 *   ESPERANDO_VACIO  los LEDs se apagan. El servo sigue en desvio hasta que
 *                    la lectura baje de 40 g (bandeja nueva). En la planta la
 *                    bascula conmuta sola a la bandeja vacia; en Tinkercad se
 *                    simula bajando el control del sensor.
 *
 * INTEGRIDAD
 * Un pico de vibracion no desvia la tolva: hace falta que el peso filtrado
 * permanezca en 500 g durante T_SOSTENER_MS. Si la lectura salta y no se
 * sostiene, se descarta.
 * =============================================================================
 */

#include <Servo.h>
#include <LiquidCrystal.h>
#include <Adafruit_NeoPixel.h>

const int PIN_FUERZA = A0;
const int PIN_SERVO = 9;
const int PIN_NEOPIXEL = 6;
const int PIN_LATIDO = 8;
const int NUM_LEDS = 16;

const int PESO_OBJETIVO_G = 500;   // SKU: bandeja de lechuga crespa, 500 g
const int ADC_CERO = 30;           // Lectura de bandeja vacia en el simulador
const int ADC_OBJETIVO = 820;      // Lectura que se hace equivaler a 500 g
const int PESO_BANDEJA_NUEVA_G = 40;
const int SERVO_DIRECTO = 0;
const int SERVO_DESVIO = 90;

const unsigned long T_SOSTENER_MS = 300;     // Antirrebote del umbral de llenado
const unsigned long T_CONFIRMACION_MS = 10000; // Ventana para capturar la evidencia
const unsigned long T_LATIDO_MS = 500;
const unsigned long T_INFORME_MS = 400;
const unsigned long T_CICLO_LARGO_MS = 180000; // Alarma de bascula quieta en cero
const int SALTO_ADC_MAX = 200;
const unsigned long T_ACEPTAR_SALTO_MS = 350;

enum EstadoLinea {
  LLENANDO,
  CONFIRMANDO,
  ESPERANDO_VACIO
};

Servo compuerta;
LiquidCrystal lcd(12, 11, 5, 4, 3, 2); // RS, E, D4, D5, D6, D7
Adafruit_NeoPixel barra(NUM_LEDS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

EstadoLinea estado = LLENANDO;
unsigned long tSobreObjetivo = 0;
unsigned long tInicioConfirmacion = 0;
unsigned long tInicioLlenado = 0;
unsigned long tLatido = 0;
unsigned long tInforme = 0;
unsigned long tInicioSalto = 0;
bool latidoEncendido = false;
int ultimoAceptado = -1;
bool alarmaBasculaYaMostrada = false;

const int VENTANA = 5;
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

// Convierte el ADC calibrado a gramos. Por encima del objetivo se informa
// el exceso, pero el porcentaje de la barra se satura en 100.
int pesoEnGramos(int adc) {
  if (adc <= ADC_CERO) {
    return 0;
  }
  long span = (long)ADC_OBJETIVO - ADC_CERO;
  long gramos = (long)(adc - ADC_CERO) * PESO_OBJETIVO_G / span;
  if (gramos < 0) {
    gramos = 0;
  }
  // Tope de presentacion: un golpe no se muestra como cientos de kilos.
  if (gramos > PESO_OBJETIVO_G + 200) {
    gramos = PESO_OBJETIVO_G + 200;
  }
  return (int)gramos;
}

int ledsPorPeso(int gramos) {
  if (gramos <= 0) {
    return 0;
  }
  int leds = (gramos * NUM_LEDS) / PESO_OBJETIVO_G;
  if (leds > NUM_LEDS) {
    leds = NUM_LEDS;
  }
  return leds;
}

void pintarBarra(int ledsEncendidos, int rojo, int verde, int azul) {
  for (int i = 0; i < NUM_LEDS; i++) {
    if (i < ledsEncendidos) {
      barra.setPixelColor(i, barra.Color(rojo, verde, azul));
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

void mostrarLlenado(int gramos, int leds) {
  char fila1[17];
  char fila2[17];
  // El producto se hace en long: en el Uno, int es de 16 bits y 400 * 100 se desborda.
  int porcentaje = (int)((long)gramos * 100L / PESO_OBJETIVO_G);
  if (porcentaje > 100) {
    porcentaje = 100;
  }
  snprintf(fila1, sizeof(fila1), "Peso %3dg %3d%%", gramos, porcentaje);
  snprintf(fila2, sizeof(fila2), "Directo  LED %02d", leds);
  escribirLcd(fila1, fila2);
}

void setup() {
  pinMode(PIN_LATIDO, OUTPUT);
  Serial.begin(9600);
  lcd.begin(16, 2);
  barra.begin();
  barra.show();
  compuerta.attach(PIN_SERVO);
  compuerta.write(SERVO_DIRECTO);
  tInicioLlenado = millis();
  escribirLcd("Linea de empaque", "Lechuga 500 g");
  Serial.println(F("ESCENARIO 2 - Pesaje y desvio - bandeja 500 g"));
  Serial.println(F("Suba el sensor de fuerza para llenar. Al llegar a 500 g"));
  Serial.println(F("el servo desvia. Baje el sensor para simular bandeja nueva."));
  delay(1200);
}

void loop() {
  int adcCrudo = analogRead(PIN_FUERZA);
  int adc = lecturaConfiable(adcCrudo);
  int gramos = pesoEnGramos(adc);
  int leds = ledsPorPeso(gramos);

  if (estado == LLENANDO) {
    compuerta.write(SERVO_DIRECTO);
    pintarBarra(leds, 0, 120, 0);

    // Sensor clavado en vacio: no se inventa el peso ni se desvia la tolva.
    bool basculaQuieta = (gramos == 0 && (millis() - tInicioLlenado) > T_CICLO_LARGO_MS);
    if (basculaQuieta) {
      escribirLcd("REVISAR BASCULA", "Sin peso medido");
      if (!alarmaBasculaYaMostrada) {
        alarmaBasculaYaMostrada = true;
        Serial.println(F("ALARMA bascula sin incremento; no se desvia"));
      }
    } else {
      mostrarLlenado(gramos, leds);
    }

    // Solo se desvia si el objetivo se sostiene. Un golpe de la tolva no basta.
    if (gramos >= PESO_OBJETIVO_G) {
      if (tSobreObjetivo == 0) {
        tSobreObjetivo = millis();
      }
      if (millis() - tSobreObjetivo >= T_SOSTENER_MS) {
        estado = CONFIRMANDO;
        tInicioConfirmacion = millis();
        compuerta.write(SERVO_DESVIO);
        pintarBarra(NUM_LEDS, 180, 180, 180);
        escribirLcd("BANDEJA COMPLETA", "Flujo desviado");
        Serial.println(F("EVENTO desvio lote=bandeja_500g servo=90"));
      }
    } else {
      tSobreObjetivo = 0;
    }

  } else if (estado == CONFIRMANDO) {
    compuerta.write(SERVO_DESVIO);
    pintarBarra(NUM_LEDS, 180, 180, 180);
    escribirLcd("BANDEJA COMPLETA", "Flujo desviado");
    if (millis() - tInicioConfirmacion >= T_CONFIRMACION_MS) {
      estado = ESPERANDO_VACIO;
      pintarBarra(0, 0, 0, 0);
      escribirLcd("RETIRE BANDEJA", "Baje el sensor");
    }
  } else {
    // Sigue desviado hasta ver la bandeja nueva. Evita reiniciar el conteo
    // encima de la misma bandeja llena y empacar de nuevo un peso ya cerrado.
    compuerta.write(SERVO_DESVIO);
    pintarBarra(0, 0, 0, 0);
    escribirLcd("RETIRE BANDEJA", "Baje el sensor");
    if (gramos <= PESO_BANDEJA_NUEVA_G) {
      estado = LLENANDO;
      tSobreObjetivo = 0;
      tInicioLlenado = millis();
      alarmaBasculaYaMostrada = false;
      compuerta.write(SERVO_DIRECTO);
      Serial.println(F("EVENTO nuevo_ciclo servo=0 peso=0"));
    }
  }

  if (millis() - tLatido >= T_LATIDO_MS) {
    tLatido = millis();
    latidoEncendido = !latidoEncendido;
    digitalWrite(PIN_LATIDO, latidoEncendido ? HIGH : LOW);
  }

  if (millis() - tInforme >= T_INFORME_MS) {
    tInforme = millis();
    Serial.print(F("ADC="));
    Serial.print(adc);
    Serial.print(F(" peso_g="));
    Serial.print(gramos);
    Serial.print(F(" leds="));
    Serial.print(leds);
    Serial.print(F(" servo="));
    Serial.print(estado == LLENANDO ? SERVO_DIRECTO : SERVO_DESVIO);
    Serial.print(F(" estado="));
    if (estado == LLENANDO) {
      Serial.println(F("LLENANDO"));
    } else if (estado == CONFIRMANDO) {
      Serial.println(F("CONFIRMANDO"));
    } else {
      Serial.println(F("ESPERANDO_VACIO"));
    }
  }
}
