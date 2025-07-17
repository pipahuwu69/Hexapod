#include "BluetoothSerial.h"
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

BluetoothSerial SerialBT;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

//definición de pines y variables
#define SERVOMIN  102   //pulso mínimo
#define SERVOMAX  492   //pulso máximo
#define SERVOFREQ 50   //frecuencia servos

const int ledPin = 2;
const int SM = 18;
bool BTState = false;
String buffer = "";
char com = '0';
int num = 0;
int SMPos = 90;
uint8_t servonum = 0;

void setup() {
  pinMode(ledPin, OUTPUT);  //define pin led
  digitalWrite(ledPin, HIGH);   //enciende led
  Serial.begin(9600);  //inicia comunicación serial
  myservo.attach(SM);   //define servo
  SerialBT.begin("ESP32-BT");   //inicia Bluetooth
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVOFREQ);  //configurar frecuencia servos
  myservo.write(SMPos);   //servo a 90
  delay(1000);
  digitalWrite(ledPin, LOW);   //apaga led
}

void loop() {
  if (SerialBT.available()){
    com = SerialBT.read();   //recibir comunicación
    if(com == 'n'){
      int num = buffer.toInt();
      if (num >= 0 && num <= 180){
        digitalWrite(ledPin, HIGH);
        SMPos = num;
        uint16_t pulso = map(SMPos, 0, 180, SERVOMIN, SERVOMAX);   //convertir ángulo a pulso
        pwm.setPWM(servonum, 0, pulso);   //enviar señal 'pulso' a servo n° 'servonum'
        digitalWrite(ledPin, LOW);
      }
      buffer = "";
    } else if (com == 'c'){
      servonum = buffer.toInt();
      buffer = "";
    } else if (com == 't'){
      int time = buffer.toInt();
      digitalWrite(ledPin, HIGH);
      delay(time);
      digitalWrite(ledPin, LOW);
      buffer = "";
    } else if (com == 's'){
      BTState = true;
    } else {
      buffer += com;
    }    
  } else if (BTState == false){   //parpadeo rápido antes de conectar
    digitalWrite(ledPin, HIGH);
    delay(500);
    digitalWrite(ledPin, LOW);
    delay(500);
  }
  //Menú Serial
  Serial.println("-----------");
  Serial.print("com: ");
  Serial.println(com);
  Serial.print("SMPos: ");
  Serial.println(SMPos);
  Serial.print("buffer: ");
  Serial.println(buffer);
  Serial.print("num: ");
  Serial.println(num);
  Serial.print("servonum: ");
  Serial.println(servonum);
}
