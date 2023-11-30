// NOTA: VELOCIDAD ES EN RAD/S
// CAMBIOS
// BluetoothSerial: libreria para mandar datos a ESP32
// Tiempo de muestro: controlar la frecuencia con la que se envían las mediciones de velocidades angulares a través de Bluetooth
// Comunicacion serial: permite recibir datos a través de Bluetooth
// Driver L298N: Se agregaron definiciones de pines y configuraciones para el control de motores utilizando el driver L298N.
//// Se añadieron funciones (parar, giroAntihorario, giroHorario) para controlar la dirección y velocidad de cada motor.
// Canales PWM (channelMotorR y channelMotorL)
// Variables de Control: Se añadieron variables para controlar la velocidad (cvR y cvL) y la velocidad angular (wR y wL) de cada motor.
// constValue: Se agregó la constante para calcular las velocidades angulares a partir de los pulsos del encoder.
// Se anadiero configuraciones de pines para los motores (IN1, IN2, IN3, IN4, ENA, ENB)
// Recepción y Procesamiento de Datos: Se agregó un bloque en el bucle loop() para recibir y procesar datos a través de Bluetooth, controlando así las velocidades de los motores.

#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;  // Inicialización de la comunicación Bluetooth

///////////////////// COMUNICACION SERIAL ////////////////
String inputString = "";
bool stringComplete = false;
const char separator = ',';
const int dataLength = 2; // Recibimos dos datos
int datos[dataLength];

//////////////////////MOTOR DERECHO/////////////////////////////// 
const int    C1R = 39;    // Entrada de la señal A del encoder C1(cable amarillo)
const int    C2R = 36;    // Entrada de la señal B del encoder C2(cable verde)

// Encoder derecho
volatile int nR = 0;       // Contador de pulsos del encoder derecho
volatile int antR = 0;     // Estado anterior del encoder derecho
volatile int actR = 0;     // Estado actual del encoder derecho

// Driver derecho
const int   ENA = 32;  
const int   IN1 = 33; 
const int   IN2 = 25; 

int channelMotorR = 0;
   
// Variables derecho
int cvR = 0; // ciclo de trabajo de pwm
double wR = 0; // radianes por segundo

//////////////////////MOTOR IZQUIERDO///////////////////////////////
const int    C1L = 35;                  // Entrada de la señal A del encoder.
const int    C2L = 34;                  // Entrada de la señal B del encoder.

// Encoder izquierdo
volatile int nL = 0;       // Contador de pulsos del encoder izquierdo
volatile int antL = 0;     // Estado anterior del encoder izquierdo
volatile int actL = 0;     // Estado actual del encoder izquierdo

// Driver izquierdo
const int IN3 = 26;  
const int IN4 = 27; 
const int ENB = 14; 

int channelMotorL = 1;

// Variables izquierdo
int cvL = 0;
double wL = 0;

/////////////////////// TIEMPO DE MUESTREO /////////////////////
unsigned long lastTime = 0, sampleTime = 100;

//////// VARIABLES PARA CALCULAR VELOCIDADES ANGULARES /////////
double constValue = 4.1975; // (1000*2*pi)/R ---> R = 1496.88 - 350RPM Resolucion encoder cuadruple

// Configuracion de las salidas pwm
const int freq = 10000;
const int resolution = 8; // 8 bits

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
  SerialBT.begin("Robot_movil_autonomo_esp32");  // Inicialización de la comunicación Bluetooth con el nombre del dispositivo

  // Configuración de los pines como entradas
  pinMode(C1R, INPUT);
  pinMode(C2R, INPUT);
  pinMode(C1L, INPUT);
  pinMode(C2L, INPUT);
  
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Configuración del driver derecho
  ledcSetup(channelMotorR,freq, resolution);
  ledcAttachPin(ENA,channelMotorR);
  
  // Configuración del driver izquierdo
  ledcSetup(channelMotorL,freq, resolution);
  ledcAttachPin(ENB,channelMotorL);

  // Detenemos ambos motores
  parar(IN1,IN2,ENA);
  parar(IN4,IN3,ENB);

  // Configuración de las interrupciones
  attachInterrupt(C1R, encoderR, CHANGE);
  attachInterrupt(C2R, encoderR, CHANGE);

  attachInterrupt(C1L, encoderL, CHANGE);
  attachInterrupt(C2L, encoderL, CHANGE);             

  lastTime = millis();  // Inicialización del tiempo de muestreo
}

void loop() 
{
  if(SerialBT.available()) serialEvent();
  
  ////////// SI RECIBE DATOS /////////////
  if (stringComplete) 
  {
    // Parseamos los datos recibidos
    for (int i = 0; i < dataLength ; i++)
    {
      int index = inputString.indexOf(separator);
      datos[i] = inputString.substring(0, index).toInt();
      inputString = inputString.substring(index + 1);
    }

    // Control de dirección y velocidad del motor derecho
    cvR = datos[0];
    if (cvR > 0) {
      giroHorario(IN1, IN2, channelMotorR, cvR);
    } else if (cvR < 0) {
      giroAntihorario(IN1, IN2, channelMotorR, abs(cvR));
    } else {
      parar(IN1, IN2, channelMotorR);
    }

    // Control de velocidad del motor izquierdo
    cvL = datos[1];
    // Control de dirección y velocidad del motor izquierdo
    if (cvL > 0) {
      giroAntihorario(IN4, IN3, channelMotorL, cvL);
    } else if (cvL < 0) {
      giroHorario(IN4, IN3, channelMotorL, abs(cvL));
    } else {
      parar(IN4, IN3, channelMotorL);
    }

    inputString = "";
    stringComplete = false;
  }

  // Cálculo y envío de velocidades angulares
  if (millis() - lastTime >= sampleTime)
  {
    wR = constValue * nR / (millis() - lastTime); // Velocidad angular del lado derecho [rad/s]
    wL = constValue * nL / (millis() - lastTime); // Velocidad angular del lado izquierdo [rad/s]
    lastTime = millis();
    nR = 0;
    nL = 0;

    SerialBT.print("Velocidad derecha: ");
    SerialBT.print(wR);
    SerialBT.print(", Velocidad izquierda: ");
    SerialBT.println(wL);
  }
}

/////////////// RECEPCION DE DATOS /////////////////////
void serialEvent() {
  while (SerialBT.available()) {
    char inChar = (char)SerialBT.read();
    inputString += inChar;
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}

// Función para detener el motor
void parar(int _in1, int _in2, int _en)
{
  digitalWrite(_in1, HIGH);
  digitalWrite(_in2, HIGH);
  digitalWrite(_en, HIGH);
}

// Función para girar en sentido antihorario
void giroAntihorario(int _in1, int _in2, int _en, int cv)
{
  digitalWrite(_in1, HIGH);
  digitalWrite(_in2, LOW);
  ledcWrite(_en, cv);
}

// Función para girar en sentido horario
void giroHorario(int _in1, int _in2, int _en, int cv)
{
  digitalWrite(_in1, LOW);
  digitalWrite(_in2, HIGH);
  ledcWrite(_en, cv);
}

