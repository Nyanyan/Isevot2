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
Servo hold_servo[2]; // right, left

#define RELAY_VOLTAGE A3
#define RELAY_POLARITY A2
#define RELAY_RIGHT A0
#define RELAY_LEFT A1

#define LED 13

// constant
#define RIGHT 0
#define LEFT 1
#define BLACK 0
#define WHITE 1

// relay
#define VOLTAGE_12V LOW
#define VOLTAGE_5V HIGH
const bool POLARITY[2][2] = {
  {true, false}, // right black, right white
  {false, true}  // left black, left white
}

// pwm servo deg
const int HOLD_DEG_OPEN[2] = {50, 50};
const int HOLD_DEG_UP[2] = {40, 40};
const int HOLD_DEG_CLOSE[2] = {70, 70};
#define VERTICAL_DEG_UP 40
#define VERTICAL_DEG_BOARD 80
#define VERTICAL_DEG_SUPPLY 50

void setup() {
  // Software Serial for PC
  Serial2.begin(9600);
  
  // Hardware Serial for Servo
  krs.begin();

  // PWM servo
  servo_vertical.attach(SERVO_VERTICAL);
  hold_servo[RIGHT].attach(SERVO_RIGHT);
  hold_servo[LEFT].attach(SERVO_LEFT);

  // Relay for electromagnets
  pinMode(RELAY_VOLTAGE, OUTPUT);
  pinMode(RELAY_POLARITY, OUTPUT);
  pinMode(RELAY_RIGHT, OUTPUT);
  pinMode(RELAY_LEFT, OUTPUT);

  pinMode(LED, OUTPUT);
  
  Serial2.println("Start!");
  digitalWrite(LED, HIGH);
}

void move_vertical(int now_deg, int to_deg, int delay_microsec_per_deg) {
  int deg_dif = abs(now_deg - to_deg);
  int delta = 1;
  if (to_deg < now_deg) {
    delta = -1;
  }
  for (int deg = now_deg; deg != to_deg; deg += delta) {
    servo_vertical.write(deg);
    delayMicroseconds(delay_microsec_per_deg);
  }
}

void lower_arm(int rl) {
  hold_servo[rl].write(HOLD_DEG_OPEN[rl]);
  hold_servo[rl ^ 1].write(HOLD_DEG_UP[rl ^ 1]);
  delay(100);
  move_vertical(VERTICAL_DEG_UP, VERTICAL_DEG_BOARD, 50);
}

void raise_arm() {
  move_vertical(VERTICAL_DEG_BOARD, VERTICAL_DEG_UP, 50);
}

void hold_disc_board(int rl, int bw) {
  digitalWrite(RELAY_VOLTAGE, VOLTAGE_12V);
  digitalWrite(RELAY_POLARITY, POLARITY[rl][bw]);
  if (rl == RIGHT) {
    digitalWrite(RELAY_RIGHT, HIGH);
    digitalWrite(RELAY_LEFT, LOW);
  } else {
    digitalWrite(RELAY_RIGHT, LOW);
    digitalWrite(RELAY_LEFT, HIGH);
  }
}

void flip_disc(int rl_from, int bw_from) {
  for (int i = 0; i < 2; ++i) {
    hold_servo[i].write(HOLD_DEG_CLOSE[i]);
  }
  if (rl_from == RIGHT) {
    digitalWrite(RELAY_LEFT, HIGH);
  } else {
    digitalWrite(RELAY_RIGHT, HIGH);
  }
  delay(50);
  if (rl_from == RIGHT) {
    digitalWrite(RELAY_RIGHT, LOW);
  } else {
    digitalWrite(RELAY_LEFT, LOW);
  }
  delay(50);
  for (int i = 0; i < 2; ++i) {
    hold_servo[i].write(HOLD_DEG_OPEN[i]);
  }
}

void move_arm(int x_mm, int y_mm, int delay_microsec_per_deg) {

}



void loop() {
  servo_vertical.write(90);
  delay(500);
  servo_vertical.write(100);
  delay(500);
}
