#include <Arduino.h>
#include "BluetoothSerial.h"

TaskHandle_t TaskBlutu;
TaskHandle_t TaskControl;

hw_timer_t *timerGiro = NULL;
hw_timer_t *timerMuestreo = NULL;

volatile bool cambiarSentido = false;
volatile bool muestrear = false;
static bool sentido = true;
bool activar = false;   // 🔥 NUEVA BANDERA: habilita el robot

#define PWMA_X 15
#define AIN1_X 2
#define AIN2_X 4


#define BIN1_X 16
#define BIN2_X 17
#define PWMB_X 5
#define STBY_X 21

/*
#define AIN1_Y 4
#define AIN2_Y 2
#define PWMA_Y 15
#define BIN1_Y 19
#define BIN2_Y 18
#define PWMB_Y 5
#define STBY_Y 21
*/

const int pinADC1 = 34;
const int pinADC2 = 35;
const int pinADC3 = 32;

int raw1 = 0;
int raw2 = 0;
int raw3 = 0;

float vt1 = 0;
float vt2 = 0;
float vt3 = 0;

float referencia = 8.75;
float angulo = 0;
float error = 0;
float sigU = 0;
float uk = 0;         
float pwmbase = 130;
float pwbt = 0;

String inputString = "";


// --------- FUNCIONES OMITIDAS POR BREVIDAD ---------

//estos dos van con x osea un puente H
void movermotor1DE(float pw)
{
  // --- Motor 1 --- (BASE)
  digitalWrite(AIN1_X, LOW);
  digitalWrite(AIN2_X, HIGH);
  analogWrite(PWMA_X,pw);
}

void movermotor1IZ(float pw)
{
  // --- Motor 1 --- (BASE)
  digitalWrite(AIN1_X, HIGH);
  digitalWrite(AIN2_X, LOW);
  analogWrite(PWMA_X,pw);
}

void movermotor1STOP(float pw)
{
  // --- Motor 1 --- (BASE)
  digitalWrite(AIN1_X, LOW);
  digitalWrite(AIN2_X, LOW);
  analogWrite(PWMA_X,pw);
}

void movermotor2STOP(float pw)
{
  // --- Motor 2 --- (CODO)
  digitalWrite(BIN1_X, LOW);
  digitalWrite(BIN2_X, LOW);
  analogWrite(PWMB_X,pw);
}

void movermotor2DE(float pw)
{
  // --- Motor 2 --- (CODO)
  digitalWrite(BIN1_X, LOW);
  digitalWrite(BIN2_X, HIGH);
  analogWrite(PWMB_X,pw);
}

void movermotor2IZ(float pw)
{
  // --- Motor 2 --- (CODO)
  digitalWrite(BIN1_X, HIGH);
  digitalWrite(BIN2_X, LOW);
  analogWrite(PWMB_X,pw);
}

// --------- TIMERS ISR ---------
void IRAM_ATTR onTimerGiro() { cambiarSentido = true; }
void IRAM_ATTR onTimerMuestreo() { muestrear = true; }

// --------- TAREA BLUETOOTH ---------
void loopBlutuchi(void *parameter){
  for(;;){
    
    Serial.print(vt1);
    Serial.print(",");
    Serial.print(vt2);
    Serial.print(",");
    Serial.println(vt3);

    if (Serial.available()){
    inputString = Serial.readStringUntil('\n');
    inputString.trim();
    }

    delay(200);
  }
}

// --------- TAREA CONTROL ---------
void loopControl(void *parameter){
  for(;;){
    if (inputString == "0,1") { 
      // Si no está activado, mantener motores apagados
      movermotor1DE(255);
      movermotor2DE(255);
      //digitalWrite(AIN2_X,LOW);
    }
    else if (inputString == "1,0") {
      movermotor1IZ(255);
      movermotor2IZ(255);
      //digitalWrite(AIN2_X,HIGH);
    }
    else if (inputString == "0,0") {
      movermotor1STOP(0);
      movermotor2STOP(0);
      //digitalWrite(AIN2_X,HIGH);
    }

    if (muestrear) {
      
      muestrear = false;
      raw1= analogRead(pinADC1);
      raw2= analogRead(pinADC2);
      raw3= analogRead(pinADC3);

      vt1 = (raw1 / 4095.0) * 3.3;
      vt2 = (raw2 / 4095.0) * 3.3;
      vt3 = (raw3 / 4095.0) * 3.3;
      
    }

    
  }
}

// --------- SETUP ---------
void setup() {
  
  Serial.begin(115200);

  analogReadResolution(12);  
  analogSetAttenuation(ADC_11db);  

  xTaskCreatePinnedToCore(loopBlutuchi,"TaskBlutu",4096,NULL,1,&TaskBlutu,0);
  xTaskCreatePinnedToCore(loopControl,"TaskControl",4096,NULL,1,&TaskControl,1);

  pinMode(AIN1_X, OUTPUT); pinMode(AIN2_X, OUTPUT); pinMode(PWMA_X, OUTPUT);
  pinMode(BIN1_X, OUTPUT); pinMode(BIN2_X, OUTPUT); pinMode(PWMB_X, OUTPUT);
  pinMode(STBY_X, OUTPUT);

  timerGiro = timerBegin(0, 80, true);
  timerAttachInterrupt(timerGiro, &onTimerGiro, true);
  timerAlarmWrite(timerGiro, 600000, true); // 0.5s
  timerAlarmEnable(timerGiro);

  timerMuestreo = timerBegin(1, 80, true);
  timerAttachInterrupt(timerMuestreo, &onTimerMuestreo, true);
  timerAlarmWrite(timerMuestreo, 2000, true); // 2ms
  timerAlarmEnable(timerMuestreo);

  digitalWrite(STBY_X, HIGH);
}

void loop() {}