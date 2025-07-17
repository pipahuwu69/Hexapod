#include "BluetoothSerial.h"
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

BluetoothSerial SerialBT;

#define SERVOMIN  102   // Pulso mínimo (~500 us)
#define SERVOMAX  492   // Pulso máximo (~2400 us)
#define SERVOFREQ 50    // Frecuencia típica de servo (Hz)

Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x40); // Placa 1
Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x41); // Placa 2

const int ledPin = 2;

String buffer = "";
char com = '0';
int angle = 90;
uint8_t servonum = 0;
bool BTState = false;

void setup() {
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);

  Serial.begin(9600);
  SerialBT.begin("ESP32-BT");

  // Inicializa ambas placas
  pwm1.begin();
  pwm1.setOscillatorFrequency(27000000);
  pwm1.setPWMFreq(SERVOFREQ);

  pwm2.begin();
  pwm2.setOscillatorFrequency(27000000);
  pwm2.setPWMFreq(SERVOFREQ);

  delay(1000);
  digitalWrite(ledPin, LOW);
}

void loop() {
  if (SerialBT.available()) {
    com = SerialBT.read();
    if (com == 'n') {
      angle = buffer.toInt();
      if (angle >= 0 && angle <= 180) {
        uint16_t pulse = map(angle, 0, 180, SERVOMIN, SERVOMAX);
        if (servonum < 16) {
          pwm1.setPWM(servonum, 0, pulse);
        } else if (servonum < 32) {
          pwm2.setPWM(servonum - 16, 0, pulse);
        }
        digitalWrite(ledPin, HIGH);
        delay(100);
        digitalWrite(ledPin, LOW);
      }
      buffer = "";
    } else if (com == 'c') {
      servonum = buffer.toInt();
      if (servonum > 31) servonum = 31; // Límite máximo
      buffer = "";
    } else if (com == 't') {
      int time = buffer.toInt();
      digitalWrite(ledPin, HIGH);
      delay(time);
      digitalWrite(ledPin, LOW);
      buffer = "";
    } else if (com == 's') {
      BTState = true;
    } else {
      buffer += com;
    }
  } else if (!BTState) {
    digitalWrite(ledPin, HIGH);
    delay(300);
    digitalWrite(ledPin, LOW);
    delay(300);
  }

  // Solo para depurar
  Serial.println("----- Estado -----");
  Serial.print("Comando: "); Serial.println(com);
  Serial.print("Buffer: "); Serial.println(buffer);
  Serial.print("Servo actual: "); Serial.println(servonum);
  Serial.print("Ángulo: "); Serial.println(angle);
}
