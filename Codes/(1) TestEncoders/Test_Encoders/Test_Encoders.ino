//////////////////////MOTOR DERECHO/////////////////////////////// 
const int    C1R = 39;    // Entrada de la señal A del encoder C1(cable amarillo)
const int    C2R = 36;    // Entrada de la señal B del encoder C2(cable verde)

// Encoder derecho
volatile int nR = 0;       // Contador de pulsos del encoder derecho
volatile int antR = 0;     // Estado anterior del encoder derecho
volatile int actR = 0;     // Estado actual del encoder derecho


//////////////////////MOTOR IZQUIERDO///////////////////////////////
const int    C1L = 35;     // Entrada de la señal A del encoder C1(cable amarillo)
const int    C2L = 34;     // Entrada de la señal B del encoder C2(cable verde)

// Encoder izquierdo
volatile int nL = 0;       // Contador de pulsos del encoder izquierdo
volatile int antL = 0;     // Estado anterior del encoder izquierdo
volatile int actL = 0;     // Estado actual del encoder izquierdo


/////////////////////// TIEMPO DE MUESTREO /////////////////////
unsigned long lastTime = 0, sampleTime = 100;  // Último tiempo de muestreo y tiempo de muestreo en milisegundos


/////////////////////// INTERRUPCIONES //////////////////////
void IRAM_ATTR encoderR()
{
  antR = actR;  // Actualizamos el estado anterior

  // Leemos las señales A y B del encoder derecho y actualizamos el estado actual
  if (digitalRead(C2R)) bitSet(actR, 0); else bitClear(actR, 0);
  if (digitalRead(C1R)) bitSet(actR, 1); else bitClear(actR, 1);

  // Lógica para determinar el sentido de giro y actualizar el contador de pulsos
  if (antR == 2 && actR == 0) nR--;
  if (antR == 0 && actR == 1) nR--;
  if (antR == 3 && actR == 2) nR--;
  if (antR == 1 && actR == 3) nR--;

  if (antR == 1 && actR == 0) nR++;
  if (antR == 3 && actR == 1) nR++;
  if (antR == 0 && actR == 2) nR++;
  if (antR == 2 && actR == 3) nR++;
}

void IRAM_ATTR encoderL()
{
  antL = actL;  // Actualizamos el estado anterior

  // Leemos las señales A y B del encoder izquierdo y actualizamos el estado actual
  if (digitalRead(C2L)) bitSet(actL, 0); else bitClear(actL, 0);
  if (digitalRead(C1L)) bitSet(actL, 1); else bitClear(actL, 1);

  // Lógica para determinar el sentido de giro y actualizar el contador de pulsos
  if (antL == 2 && actL == 0) nL++;
  if (antL == 0 && actL == 1) nL++;
  if (antL == 3 && actL == 2) nL++;
  if (antL == 1 && actL == 3) nL--;

  if (antL == 1 && actL == 0) nL--;
  if (antL == 3 && actL == 1) nL--;
  if (antL == 0 && actL == 2) nL--;
  if (antL == 2 && actL == 3) nL--;
}

void setup()
{
  Serial.begin(9600);  // Inicialización de la comunicación serial

  // Configuración de los pines como entradas
  pinMode(C1R, INPUT);
  pinMode(C2R, INPUT);
  pinMode(C1L, INPUT);
  pinMode(C2L, INPUT);

  // Habilitamos interrupciones como entradas
  attachInterrupt(C1R, encoderR, CHANGE);
  attachInterrupt(C2R, encoderR, CHANGE);

  attachInterrupt(C1L, encoderL, CHANGE);
  attachInterrupt(C2L, encoderL, CHANGE);

  lastTime = millis();  // Inicialización del tiempo de muestreo
}

void loop()
{
  // Verifica si ha pasado el tiempo de muestreo
  if (millis() - lastTime >= sampleTime)
  {
    lastTime = millis();  // Actualiza el último tiempo de muestreo
    // Imprime los valores de los contadores de pulsos en la consola serial
    Serial.print("Derecha: ");
    Serial.print(nR);
    Serial.print(", Izquierda: ");
    Serial.println(nL);
  }
}
