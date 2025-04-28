#include <SoftwareSerial.h>
#include <IcsHardSerialClass.h>
#include <Servo.h>

//SoftwareSerial Serial2(3, 2);

#define EN_PIN 7
#define BAUDRATE 115200
#define TIMEOUT 1000

IcsHardSerialClass krs(&Serial, EN_PIN, BAUDRATE, TIMEOUT);

#define SERVO_VERTICAL 9
#define SERVO_RIGHT 10
#define SERVO_LEFT 11

Servo servo_vertical, servo_right, servo_left;

#define RELAY_VOLTAGE A3
#define RELAY_POLARITY A2
#define RELAY_LEFT A1
#define RELAY_RIGHT A0

#define LED 13

void setup() {
  // Software Serial for PC
  Serial2.begin(9600);
  
  // Hardware Serial for Servo
  krs.begin();

  // PWM servo
  servo_vertical.attach(SERVO_VERTICAL);
  servo_left.attach(SERVO_LEFT);
  servo_right.attach(SERVO_RIGHT);

  // Relay for electromagnets
  pinMode(RELAY_VOLTAGE, OUTPUT);
  pinMode(RELAY_POLARITY, OUTPUT);
  pinMode(RELAY_LEFT, OUTPUT);
  pinMode(RELAY_RIGHT, OUTPUT);

  pinMode(LED, OUTPUT);
  
  Serial2.println("Start!");
  digitalWrite(LED, HIGH);
}

void loop() {
  servo_vertical.write(90);
  delay(500);
  servo_vertical.write(100);
  delay(500);
}
