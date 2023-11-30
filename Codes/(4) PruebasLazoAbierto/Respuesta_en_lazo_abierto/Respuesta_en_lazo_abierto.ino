// CAMBIOS
// -----------------------------------------------------------------------------------------------------------------------------------------------------------------
// Libreria motorControl.h: contiene funciones y clases relacionadas con el control de motores
// Objetos de Control de Motor: Se crean dos objetos motorControl, uno para el motor derecho (motorR) y otro para el motor izquierdo (motorL). 
    //// Estos objetos se inicializan con el tiempo de muestreo (sampleTime), que probablemente esté relacionado con la frecuencia de muestreo del sistema de control.
// Límites de Señales: Se establecen límites para las señales de control (Cv) y las variables de proceso (Pv) para ambos motores mediante las funciones setCvLimits 
    //// y setPvLimits de los objetos motorControl. Estos límites estan relacionados con el rango de velocidad y otros parámetros del sistema.
// Escalado de Señales: Antes de aplicar las señales de control a los motores, se realiza un escalado de las señales mediante las funciones scaleCv y scalePv de los 
    //// objetos motorControl. Esto puede ser necesario para adaptar las señales a los rangos aceptables por los motores o el sistema de control.
// Uso de Macros para Pines: Se utiliza el símbolo #define para asignar nombres a los pines de los motores y las entradas de los encoders. 
    //// Esto hace que el código sea más legible y fácil de mantener.
// Envío de Datos Bluetooth: En lugar de simplemente enviar las velocidades angulares de los motores, se envía una cadena "E" seguida de 
    //// las velocidades angulares del motor derecho e izquierdo. 

#include "motorControl.h"   // Incluye la biblioteca motorControl para el control del motor
#include "BluetoothSerial.h"  // Incluye la biblioteca BluetoothSerial para la comunicación Bluetooth

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;  // Crea una instancia de BluetoothSerial llamada SerialBT

/////////////////////// TIEMPO DE MUESTREO /////////////////////
unsigned long lastTime = 0, sampleTime = 100;  // Definición de variables para el tiempo de muestreo

///////////////////// COMUNICACION SERIAL ////////////////
String inputString = "";  // Cadena de entrada para datos serie
bool stringComplete = false;  // Bandera para indicar que se ha completado una cadena
const char separator = ',';  // Separador para los datos en la cadena
const int dataLength = 2;  // Número de datos esperados en la cadena
int datos[dataLength];  // Arreglo para almacenar los datos

//////////////////////MOTOR DERECHO/////////////////////////////// 
motorControl motorR(sampleTime);  // Crea una instancia de motorControl para el motor derecho con el tiempo de muestreo
const int C1R = 39;    // Entrada de la señal A del encoder C1(cable amarillo)
const int C2R = 36;    // Entrada de la señal B del encoder C2(cable verde)

// Encoder
volatile int nR = 0;
volatile int antR = 0;
volatile int actR = 0;

// Driver
const int ENA = 32;  
const int IN1 = 33; 
const int IN2 = 25; 

int channelMotorR = 0;

// Variables
int cvR = 0;
double wR = 0;

//////////////////////MOTOR IZQUIERDO///////////////////////////////
motorControl motorL(sampleTime);  // Crea una instancia de motorControl para el motor izquierdo con el tiempo de muestreo
const int C1L = 35;  // Entrada de la señal A del encoder.
const int C2L = 34;  // Entrada de la señal B del encoder.

// Encoder
volatile int nL = 0;
volatile int antL = 0;
volatile int actL = 0; 

// Driver
const int IN3 = 26;  
const int IN4 = 27; 
const int ENB = 14; 

int channelMotorL = 1;

// Variables
int cvL = 0;
double wL = 0;

//////// VARIABLES PARA CALCULAR VELOCIDADES ANGULARES /////////
double constValue = 4.1975; // (1000*2*pi)/R ---> R = 1496.88 - 350RPM Resolucion encoder cuádruple

// Configuracion de las salidas pwm
const int freq = 10000;
const int resolution = 8;

/////////////////////// INTERRUPCIONES //////////////////////
void IRAM_ATTR encoderR()
{
  antR = actR;
               
  if(digitalRead(C2R)) bitSet(actR,0); else bitClear(actR,0);
  if(digitalRead(C1R)) bitSet(actR,1); else bitClear(actR,1);
  
  if(antR == 2 && actR ==0) nR--;
  if(antR == 0 && actR ==1) nR--;
  if(antR == 3 && actR ==2) nR--;
  if(antR == 1 && actR ==3) nR--;
  
  if(antR == 1 && actR ==0) nR++;
  if(antR == 3 && actR ==1) nR++;
  if(antR == 0 && actR ==2) nR++;
  if(antR == 2 && actR ==3) nR++;    
}

void IRAM_ATTR encoderL()
{
  antL = actL;
               
  if(digitalRead(C2L)) bitSet(actL,0); else bitClear(actL,0);
  if(digitalRead(C1L)) bitSet(actL,1); else bitClear(actL,1);
  
  if(antL == 2 && actL ==0) nL++;
  if(antL == 0 && actL ==1) nL++;
  if(antL == 3 && actL ==2) nL++;
  if(antL == 1 && actL ==3) nL++;
  
  if(antL == 1 && actL ==0) nL--;
  if(antL == 3 && actL ==1) nL--;
  if(antL == 0 && actL ==2) nL--;
  if(antL == 2 && actL ==3) nL--;     
}

void setup()
{
  SerialBT.begin("Robot_movil_autonomo_esp32");  // Inicializa la comunicación Bluetooth

  // Configuracion de los pines
  pinMode(C1R, INPUT);
  pinMode(C2R, INPUT);
  pinMode(C1L, INPUT);
  pinMode(C2L, INPUT);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Driver
  ledcSetup(channelMotorR, freq, resolution);
  ledcAttachPin(ENA, channelMotorR);
  
  ledcSetup(channelMotorL, freq, resolution);
  ledcAttachPin(ENB, channelMotorL);

  parar(IN1, IN2, ENA);
  parar(IN4, IN3, ENB);

  // Interrupciones
  attachInterrupt(C1R, encoderR, CHANGE);
  attachInterrupt(C2R, encoderR, CHANGE);
  attachInterrupt(C1L, encoderL, CHANGE);
  attachInterrupt(C2L, encoderL, CHANGE); 

  ////////////////// Limites de señales //////////////////
  motorR.setCvLimits(255, 0);
  motorR.setPvLimits(19, 0);
  
  ////////////////// Limites de señales //////////////////
  motorL.setCvLimits(255, 0);
  motorL.setPvLimits(19, 0);
  
  lastTime = millis(); 
}

void loop() 
{
  if(SerialBT.available()) serialEvent();
  
  ////////// SI RECIBE DATOS /////////////
  if (stringComplete) 
  {
    for (int i = 0; i < dataLength ; i++)
    {
      int index = inputString.indexOf(separator);
      datos[i] = inputString.substring(0, index).toInt();
      inputString = inputString.substring(index + 1);
    }

    // Escalamos y controlamos el motor derecho
    cvR = motorR.scaleCv(datos[0]);
    if(cvR > 0) giroHorario(IN1, IN2, channelMotorR, cvR);
    else if (cvR < 0) giroAntihorario(IN1, IN2, channelMotorR, abs(cvR));
    else parar(IN1, IN2, channelMotorR);

    // Escalamos y controlamos el motor izquierdo
    cvL = motorL.scaleCv(datos[1]);
    if(cvL > 0) giroAntihorario(IN4, IN3, channelMotorL, cvL);
    else if (cvL < 0) giroHorario(IN4, IN3, channelMotorL, abs(cvL));
    else parar(IN4, IN3, channelMotorL);

    inputString = "";
    stringComplete = false;
  }

  if(millis()-lastTime >= sampleTime)
  {
    wR = constValue * nR / (millis() - lastTime); // Velocidad angular del lado derecho [rad/s]
    wL = constValue * nL / (millis() - lastTime); // Velocidad angular del lado izquierdo [rad/s]
    lastTime = millis();

    wR = motorR.scalePv(wR);  // Escalamos la velocidad angular derecha
    wL = motorL.scalePv(wL);  // Escalamos la velocidad angular izquierda
    
    nR = 0;
    nL = 0;
    
    SerialBT.println("E");
    SerialBT.println(wR);
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

void parar(int _in1, int _in2, int _en)
{
  digitalWrite(_in1, HIGH);
  digitalWrite(_in2, HIGH);
  digitalWrite(_en, HIGH);
}

void giroAntihorario(int _in1, int _in2, int _en, int cv)
{   
  digitalWrite(_in1, HIGH);
  digitalWrite(_in2, LOW);
  ledcWrite(_en, cv);
}

void giroHorario(int _in1, int _in2, int _en, int cv)
{   
  digitalWrite(_in1, LOW);
  digitalWrite(_in2, HIGH);
  ledcWrite(_en, cv);
}
