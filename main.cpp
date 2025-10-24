#include <Arduino.h>
#include "BluetoothSerial.h"

TaskHandle_t TaskBlutu;
TaskHandle_t TaskControl;

hw_timer_t *timerGiro = NULL;
hw_timer_t *timerMuestreo = NULL;

volatile bool cambiarSentido = false;
volatile bool muestrear = false;
static bool sentido = true;
bool activar = false;   // Bandera habilita el robot



// Pines ADC1 (solo entrada)
#define PIN_VN   39   // SENSOR_VN  → ADC1_CH3
#define PIN_34   34   // GPIO34     → ADC1_CH6
#define PIN_35   35   // GPIO35     → ADC1_CH7
#define PIN_32   32   // GPIO32     → ADC1_CH4


// Pines 1DOF(Base), 2DOF
#define PWMA_X 21
#define AIN1_X 19
#define AIN2_X 18
#define BIN3_X 17
#define BIN4_X 16
#define PWMB_X 4

// Pines 3DOF , 4DOF
#define PWMA_Y 33
#define AIN1_Y 25
#define AIN2_Y 26
#define BIN3_Y 27
#define BIN4_Y 14
#define PWMB_Y 13



int raw1 = 0;
int raw2 = 0;
int raw3 = 0;
int raw4 = 0;


float vt1 = 0;
float vt2 = 0;
float vt3 = 0;
float vt4 = 0;


float referencia = 8.75;
float angulo = 0;
float error = 0;
float sigU = 0;
float uk = 0;         
float pwmbase = 130;
float pwbt = 0;

String inputString = "";



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

///falta mover sentido contrario 

void movermotor1STOP(float pw)
{
  // --- Motor 1 --- (BASE)
  digitalWrite(AIN1_X, LOW);
  digitalWrite(AIN2_X, LOW);
  analogWrite(PWMA_X,pw);
}

void movermotor2DE(float pw)
{
  // --- Motor 2 --- (CODO)
  digitalWrite(BIN3_X, LOW);
  digitalWrite(BIN4_X, HIGH);
  analogWrite(PWMB_X,pw);
}

void movermotor2IZ(float pw)
{
  // --- Motor 2 --- (CODO)
  digitalWrite(BIN3_X, HIGH);
  digitalWrite(BIN4_X, LOW);
  analogWrite(PWMB_X,pw);
}

void movermotor2STOP(float pw)
{
  // --- Motor 2 --- (CODO)
  digitalWrite(BIN3_X, LOW);
  digitalWrite(BIN4_X, LOW);
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
      raw1= analogRead(PIN_VN);
      raw2= analogRead(PIN_34);
      raw3= analogRead(PIN_VN);
      raw4= analogRead(PIN_34);

      vt1 = (raw1 / 4095.0) * 3.3;
      vt2 = (raw2 / 4095.0) * 3.3;
      vt3 = (raw3 / 4095.0) * 3.3;
      vt4 = (raw4 / 4095.0) * 3.3;

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
  pinMode(BIN3_X, OUTPUT); pinMode(BIN4_X, OUTPUT); pinMode(PWMB_X, OUTPUT);

  pinMode(AIN1_Y, OUTPUT); pinMode(AIN2_Y, OUTPUT); pinMode(PWMA_Y, OUTPUT);
  pinMode(BIN3_Y, OUTPUT); pinMode(BIN4_Y, OUTPUT); pinMode(PWMB_Y, OUTPUT);

  timerGiro = timerBegin(0, 80, true);
  timerAttachInterrupt(timerGiro, &onTimerGiro, true);
  timerAlarmWrite(timerGiro, 600000, true); // 0.5s
  timerAlarmEnable(timerGiro);

  timerMuestreo = timerBegin(1, 80, true);
  timerAttachInterrupt(timerMuestreo, &onTimerMuestreo, true);
  timerAlarmWrite(timerMuestreo, 2000, true); // 2ms
  timerAlarmEnable(timerMuestreo);


}

void loop() {}
