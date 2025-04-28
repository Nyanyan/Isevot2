#include <SoftwareSerial.h>
#include <IcsHardSerialClass.h>
#include <Servo.h>

SoftwareSerial Serial2(3, 2);

// krs servo
#define EN_PIN 7
#define BAUDRATE 115200
#define TIMEOUT 1000
IcsHardSerialClass krs(&Serial, EN_PIN, BAUDRATE, TIMEOUT);
#define KRS_ID_ROOT 0
#define KRS_ID_ELBOW 1
#define KRS_ID_DISC_SUPPLY 2

// pwm servo
#define SERVO_VERTICAL 9
#define SERVO_RIGHT 10
#define SERVO_LEFT 11

Servo servo_vertical, servo_right, servo_left;
Servo hold_servo[2]; // right, left

#define RELAY_VOLTAGE A3
#define RELAY_POLARITY A2
#define RELAY_RIGHT A1
#define RELAY_LEFT A0

#define LED 13

// constant
#define RIGHT 0
#define LEFT 1
#define BLACK 0
#define WHITE 1
#define HW 8
#define HW2 64

#define PI 3.141592653589793

// relay
#define VOLTAGE_12V LOW
#define VOLTAGE_5V HIGH
const bool POLARITY[2][2] = {
  {true, false}, // right black, right white
  {false, true}  // left black, left white
};

// pwm servo deg (right, left)
const int HOLD_DEG_UP[2] = {162, 23};
const int HOLD_DEG_OPEN[2] = {142, 43};
const int HOLD_DEG_CLOSE[2] = {40, 150};
#define VERTICAL_DEG_UP 20
#define VERTICAL_DEG_SUPPLY 45
#define VERTICAL_DEG_BOARD 176

// krs servo neutral position
#define KRS_NEUTRAL_ROOT 7500
#define KRS_NEUTRAL_ELBOW 6157
#define KRS_NEUTRAL_DISC_SUPPLY 4833

// hardware constant length (mm) / deg
#define LEN_ARM_ROOT 185.000
#define LEN_ARM_ELBOW 189.392
#define ELBOW_FINGER_DEG 12.3632
#define LEN_ROBOT_TO_BOARD_Y 144.424
#define LEN_CENTER_TO_CELL_X 15.750
#define LEN_CELL_SIZE 29.500
#define DISC_SUPPLY_X 157.719
#define DISC_SUPPLY_Y 7.674

void setup() {
  // Software Serial for PC
  Serial2.begin(9600);
  
  // Hardware Serial for Servo
  krs.begin();
  //Serial.begin(115200);

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

  // initialize
  servo_vertical.write(VERTICAL_DEG_UP);
  for (int i = 0; i < 2; ++i) {
    hold_servo[i].write(HOLD_DEG_OPEN[i]);
  }
  
  delay(1000);
  Serial.println("Start!");
  digitalWrite(LED, HIGH);
}

void move_vertical(int now_deg, int to_deg, int delay_msec_per_deg) {
  int deg_dif = abs(now_deg - to_deg);
  int delta = 1;
  if (to_deg < now_deg) {
    delta = -1;
  }
  for (int deg = now_deg; deg != to_deg; deg += delta) {
    servo_vertical.write(deg);
    delay(delay_msec_per_deg);
  }
  delay(10);
}

void lower_arm(int rl) {
  hold_servo[rl].write(HOLD_DEG_OPEN[rl]);
  hold_servo[rl ^ 1].write(HOLD_DEG_UP[rl ^ 1]);
  delay(50);
  servo_vertical.write(VERTICAL_DEG_BOARD);
  delay(350);
  //move_vertical(VERTICAL_DEG_UP, VERTICAL_DEG_BOARD, 2);
}

void raise_arm() {
  servo_vertical.write(VERTICAL_DEG_UP);
  delay(350);
  open_hand();
  //move_vertical(VERTICAL_DEG_BOARD, VERTICAL_DEG_UP, 2);
}

void off_relay() {
  digitalWrite(RELAY_VOLTAGE, LOW);
  digitalWrite(RELAY_POLARITY, LOW);
  digitalWrite(RELAY_RIGHT, LOW);
  digitalWrite(RELAY_LEFT, LOW);
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
  delay(200);
}

void hold_disc_supply(int rl) {
  digitalWrite(RELAY_VOLTAGE, VOLTAGE_12V);
  digitalWrite(RELAY_POLARITY, POLARITY[rl][BLACK]);
  if (rl == RIGHT) {
    digitalWrite(RELAY_RIGHT, HIGH);
    digitalWrite(RELAY_LEFT, LOW);
  } else {
    digitalWrite(RELAY_RIGHT, LOW);
    digitalWrite(RELAY_LEFT, HIGH);
  }
  delay(200);
}

void open_hand() {
  for (int i = 0; i < 2; ++i) {
    hold_servo[i].write(HOLD_DEG_OPEN[i]);
  }
}

void close_hand() {
  for (int i = 0; i < 2; ++i) {
    hold_servo[i].write(HOLD_DEG_CLOSE[i]);
  }
}

void flip_disc(int rl_from, int bw_from) {
  close_hand();
  delay(200);
  digitalWrite(RELAY_RIGHT, HIGH);
  digitalWrite(RELAY_LEFT, HIGH);
  delay(50);
  if (rl_from == RIGHT) {
    digitalWrite(RELAY_RIGHT, LOW);
  } else {
    digitalWrite(RELAY_LEFT, LOW);
  }
  delay(20);
  for (int i = 0; i < 2; ++i) {
    hold_servo[i].write(HOLD_DEG_OPEN[i]);
  }
  delay(100);
}

void put_disc(int rl, int bw) {
  digitalWrite(RELAY_RIGHT, LOW);
  digitalWrite(RELAY_LEFT, LOW);
  delay(100);
  digitalWrite(RELAY_VOLTAGE, VOLTAGE_5V);
  digitalWrite(RELAY_POLARITY, POLARITY[rl][bw ^ 1]);
  if (rl == RIGHT) {
    digitalWrite(RELAY_RIGHT, HIGH);
  } else {
    digitalWrite(RELAY_LEFT, HIGH);
  }
  raise_arm();
  off_relay();
}

int convert_to_krs_diff(double deg) { // from neutral
  return round(deg * 8000.0 / 270.0);
}

double convert_from_krs_diff(double krs_deg) { // from neutral
  return (krs_deg - 7500) * 270.0 / 8000.0;
}

void move_arm(double x_mm, double y_mm, int rl, int delay_msec) {
  double r2 = x_mm * x_mm + y_mm * y_mm;
  double r = sqrt(r2);
  const double L1 = LEN_ARM_ROOT;
  const double L2 = LEN_ARM_ELBOW;
  const double L12 = L1 * L1;
  const double L22 = L2 * L2;
  double theta2 = acos((L12 + L22 - r2) / (2.0 * L1 * L2)) * 180.0 / PI;
  double theta3 = acos((L12 - L22 + r2) / (2.0 * L1 * r)) * 180.0 / PI;
  double theta1 = acos(x_mm / r) * 180.0 / PI + theta3;
  
  Serial.print(theta1);
  Serial.print(' ');
  Serial.print(theta2);
  Serial.print(' ');
  Serial.println(theta3);

  if (rl == RIGHT) {
    theta2 -= ELBOW_FINGER_DEG;
  } else {
    theta2 += ELBOW_FINGER_DEG;
  }
  int theta1_converted = -convert_to_krs_diff(theta1 - 90.0) + KRS_NEUTRAL_ROOT;
  int theta2_converted = convert_to_krs_diff(180.0 - theta2) + KRS_NEUTRAL_ELBOW;
  
  Serial.print(theta1_converted);
  Serial.print(' ');
  Serial.println(theta2_converted);

  int theta1_start = krs.getPos(KRS_ID_ROOT);
  int theta2_start = krs.getPos(KRS_ID_ELBOW);
  Serial.print(theta1_start);
  Serial.print(' ');
  Serial.println(theta2_start);
  int diff_theta1 = theta1_converted - theta1_start;
  int diff_theta2 = theta2_converted - theta2_start;
  int step = max(abs(diff_theta1), abs(diff_theta2)) / (8000.0 / 270.0 / 2.0);
  if (step < 5) {
    step = 5;
  }
  for (int i = 0; i < step; ++i) {
    double d = (1.0 - cos(PI / step * i)) * 0.5;
    int deg1 = round((double)theta1_start + d * diff_theta1);
    int deg2 = round((double)theta2_start + d * diff_theta2);
    
    Serial.print(i);
    Serial.print(' ');
    Serial.print(d);
    Serial.print(' ');
    Serial.print(deg1);
    Serial.print(' ');
    Serial.println(deg2);

    krs.setPos(KRS_ID_ROOT, deg1);
    krs.setPos(KRS_ID_ELBOW, deg2);
    delay(delay_msec);
  }
}

void get_disc(int rl) {
  open_hand();
  int deg_get = KRS_NEUTRAL_DISC_SUPPLY + convert_to_krs_diff(180.0);
  krs.setPos(KRS_ID_DISC_SUPPLY, deg_get);
  delay(500);
  // while (abs(krs.getPos(KRS_ID_DISC_SUPPLY) - deg_get) > 40) {
  //   delay(10);
  // }
  for (int i = 0; i < 3; ++i) {
    krs.setPos(KRS_ID_DISC_SUPPLY, deg_get + 100);
    delay(60);
    krs.setPos(KRS_ID_DISC_SUPPLY, deg_get - 100);
    delay(60);
  }
  int deg_set = KRS_NEUTRAL_DISC_SUPPLY;
  krs.setPos(KRS_ID_DISC_SUPPLY, deg_set);
  delay(400);
  // while (abs(krs.getPos(KRS_ID_DISC_SUPPLY) - deg_set) > 40) {
  //   delay(10);
  // }
  move_arm(DISC_SUPPLY_X, DISC_SUPPLY_Y, rl, 5);
  servo_vertical.write(VERTICAL_DEG_SUPPLY);
  delay(100);
  hold_disc_supply(rl);
  servo_vertical.write(VERTICAL_DEG_UP);
  delay(100);
}

double calc_x_mm(int cell) {
  int x = cell % HW;
  double res = 0.0;
  if (x <= 3) { // left
    res += LEN_CENTER_TO_CELL_X;
    res += LEN_CELL_SIZE * (3 - x);
  } else { // right
    res -= LEN_CENTER_TO_CELL_X;
    res -= LEN_CELL_SIZE * (x - 4);
  }
  return res;
}

double calc_y_mm(int cell) {
  int y = cell / HW;
  double res = LEN_ROBOT_TO_BOARD_Y;
  res += LEN_CELL_SIZE * y;
  return res;
}

void set_starting_board() {
  const int cells[4] = {27, 28, 35, 36};
  const int colors[4] = {WHITE, BLACK, BLACK, WHITE};
  for (int i = 0; i < 4; ++i) {
    get_disc(RIGHT);
    if (colors[i] == BLACK) {
      move_arm(calc_x_mm(cells[i]), calc_y_mm(cells[i]), RIGHT, 5);
      lower_arm(RIGHT);
      put_disc(RIGHT, BLACK);
    } else {
      move_arm(calc_x_mm(cells[i]), calc_y_mm(cells[i]), LEFT, 5);
      flip_disc(RIGHT, BLACK);
      lower_arm(LEFT);
      put_disc(LEFT, WHITE);
    }
  }
}


void loop() {
  /*
  for (int cell = 0; cell < HW2; ++cell) {
    delay(1000);
    double x = calc_x_mm(cell);
    double y = calc_y_mm(cell);
    Serial.print(x);
    Serial.print(' ');
    Serial.println(y);
    move_arm(x, y, RIGHT, 10);
  }
  */
  //for (;;);
  /*
  servo_vertical.write(VERTICAL_DEG_UP);
  delay(1000);
  servo_vertical.write(VERTICAL_DEG_BOARD);
  delay(2000);
  */
  /*
  hold_servo[RIGHT].write(HOLD_DEG_CLOSE[RIGHT]);
  hold_servo[LEFT].write(HOLD_DEG_CLOSE[LEFT]);
  delay(1000);
  hold_servo[RIGHT].write(HOLD_DEG_OPEN[RIGHT]);
  hold_servo[LEFT].write(HOLD_DEG_OPEN[LEFT]);
  delay(2000);
  */
  
  /*
  int cell = 36;
  move_arm(calc_x_mm(cell), calc_y_mm(cell), RIGHT, 5);
  lower_arm(RIGHT);
  hold_disc_board(RIGHT, WHITE);
  raise_arm();
  flip_disc(RIGHT, WHITE);
  move_arm(calc_x_mm(cell), calc_y_mm(cell), LEFT, 5);
  lower_arm(LEFT);
  put_disc(LEFT, BLACK);
  delay(1000);
  
  move_arm(calc_x_mm(cell), calc_y_mm(cell), RIGHT, 5);
  lower_arm(RIGHT);
  hold_disc_board(RIGHT, BLACK);
  raise_arm();
  flip_disc(RIGHT, BLACK);
  move_arm(calc_x_mm(cell), calc_y_mm(cell), LEFT, 5);
  lower_arm(LEFT);
  put_disc(LEFT, WHITE);
  delay(1000);
  */


  set_starting_board();

  {
    get_disc(RIGHT);
    int cell = 37;
    move_arm(calc_x_mm(cell), calc_y_mm(cell), RIGHT, 5);
    lower_arm(RIGHT);
    put_disc(RIGHT, BLACK);
  }
  {
    int cell = 36;
    move_arm(calc_x_mm(cell), calc_y_mm(cell), RIGHT, 5);
    lower_arm(RIGHT);
    hold_disc_board(RIGHT, WHITE);
    raise_arm();
    flip_disc(RIGHT, WHITE);
    move_arm(calc_x_mm(cell), calc_y_mm(cell), LEFT, 5);
    lower_arm(LEFT);
    put_disc(LEFT, BLACK);
    delay(1000);
  }
  for (;;);
}
