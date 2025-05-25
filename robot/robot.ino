#include <SoftwareSerial.h>
#include <IcsHardSerialClass.h>
#include <Servo.h>
#include <MsTimer2.h>

SoftwareSerial Serial2(3, 2);

// krs servo
#define EN_PIN 4
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

// clock
#define PLAYER_BUTTON 8
#define TOGGLE_SWITCH 5
#define PLAYER_LED 6
#define ROBOT_LED 7

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
#define VERTICAL_DEG_CLOCK 70
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
#define DISC_SUPPLY_MODE 0
//#define HOME_RIGHT_X -112.021
//#define HOME_RIGHT_Y 75.948
#define HOME_RIGHT_X -121.794
#define HOME_RIGHT_Y 77.622
#define HOME_RIGHT_MODE 0
#define CAMERA_RIGHT_X -189.425
#define CAMERA_RIGHT_Y 75.176
#define CAMERA_RIGHT_MODE 0
#define NOPOS_X 130.0
#define NOPOS_Y 100.0
#define NOPOS_MODE 0
#define CLOCK_X 204.220
#define CLOCK_Y 210.673
#define CLOCK_MODE 0

const int arm_mode[HW2] = {
  /*
  0, 0, 0, 0, 0, 0, 0, 1, 
  0, 0, 0, 0, 0, 0, 0, 1, 
  0, 0, 0, 0, 0, 0, 0, 1, 
  0, 0, 0, 0, 0, 0, 0, 1, 
  0, 0, 0, 0, 0, 0, 0, 1, 
  0, 0, 0, 0, 0, 0, 0, 1, 
  0, 0, 0, 0, 0, 0, 0, 1, 
  0, 0, 0, 0, 0, 0, 0, 1
  */
  0, 0, 0, 0, 0, 0, 0, 0, 
  0, 0, 0, 0, 0, 0, 0, 0, 
  0, 0, 0, 0, 0, 0, 0, 0, 
  0, 0, 0, 0, 0, 0, 0, 0, 
  0, 0, 0, 0, 0, 0, 0, 0, 
  0, 0, 0, 0, 0, 0, 0, 0, 
  0, 0, 0, 0, 0, 0, 0, 0, 
  0, 0, 0, 0, 0, 0, 0, 0
};

// speed
#define KRS_SERVO_SPEED 6

// clock
#define TURN_INFO_NOT_PLAYING -1
#define TURN_INFO_PLAYER 0
#define TURN_INFO_ROBOT 1
int turn_info = TURN_INFO_NOT_PLAYING;
bool clock_led_state = false;

void blink_led() {
  clock_led_state = !clock_led_state;
  if (turn_info == TURN_INFO_PLAYER) {
    digitalWrite(PLAYER_LED, clock_led_state);
    digitalWrite(ROBOT_LED, LOW);
  } else if (turn_info == TURN_INFO_ROBOT) {
    digitalWrite(PLAYER_LED, LOW);
    digitalWrite(ROBOT_LED, clock_led_state);
  } else {
    digitalWrite(PLAYER_LED, LOW);
    digitalWrite(ROBOT_LED, LOW);
  }
}

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

  // clock
  pinMode(PLAYER_BUTTON, INPUT_PULLUP);
  pinMode(TOGGLE_SWITCH, INPUT_PULLUP);
  pinMode(PLAYER_LED, OUTPUT);
  pinMode(ROBOT_LED, OUTPUT);

  // debug LED
  pinMode(LED, OUTPUT);

  // initialize
  servo_vertical.write(VERTICAL_DEG_UP);
  for (int i = 0; i < 2; ++i) {
    hold_servo[i].write(HOLD_DEG_OPEN[i]);
  }

  MsTimer2::set(500, blink_led);
  MsTimer2::start();
  
  delay(1000);
  Serial2.println("Start!");
  digitalWrite(LED, HIGH);
  set_home();
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
  //move_vertical(VERTICAL_DEG_UP, VERTICAL_DEG_BOARD, KRS_SERVO_SPEED);
}

void raise_arm() {
  servo_vertical.write(VERTICAL_DEG_UP);
  delay(350);
  open_hand();
  //move_vertical(VERTICAL_DEG_BOARD, VERTICAL_DEG_UP, KRS_SERVO_SPEED);
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
  } else {
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

void move_arm(double x_mm, double y_mm, int rl, int delay_msec, int mode) {
  double r2 = x_mm * x_mm + y_mm * y_mm;
  double r = sqrt(r2);
  const double L1 = LEN_ARM_ROOT;
  const double L2 = LEN_ARM_ELBOW;
  const double L12 = L1 * L1;
  const double L22 = L2 * L2;
  double theta2 = acos((L12 + L22 - r2) / (2.0 * L1 * L2)) * 180.0 / PI; // elbow
  double theta3 = acos((L12 - L22 + r2) / (2.0 * L1 * r)) * 180.0 / PI;
  int theta1_converted, theta2_converted;
  if (mode == 0) {
    double theta1 = acos(x_mm / r) * 180.0 / PI + theta3; // shoulder
    if (rl == RIGHT) {
      theta2 -= ELBOW_FINGER_DEG;
    } else {
      theta2 += ELBOW_FINGER_DEG;
    }
    theta1_converted = -convert_to_krs_diff(theta1 - 90.0) + KRS_NEUTRAL_ROOT;
    theta2_converted = convert_to_krs_diff(180.0 - theta2) + KRS_NEUTRAL_ELBOW;
  } else {
    double theta1 = acos(-x_mm / r) * 180.0 / PI + theta3; // shoulder
    if (rl == RIGHT) {
      theta2 += ELBOW_FINGER_DEG;
    } else {
      theta2 -= ELBOW_FINGER_DEG;
    }
    theta1_converted = convert_to_krs_diff(theta1 - 90.0) + KRS_NEUTRAL_ROOT;
    theta2_converted = -convert_to_krs_diff(180.0 - theta2) + KRS_NEUTRAL_ELBOW;
  }
  int theta1_start = krs.getPos(KRS_ID_ROOT);
  int theta2_start = krs.getPos(KRS_ID_ELBOW);
  int diff_theta1 = theta1_converted - theta1_start;
  int diff_theta2 = theta2_converted - theta2_start;
  int step = max(abs(diff_theta1), abs(diff_theta2)) / (8000.0 / 270.0);
  int min_n_steps = 60 + round(20.0 / (double)abs(theta2_converted - KRS_NEUTRAL_ELBOW));
  if (step < min_n_steps) {
    step = min_n_steps;
  }
  for (int i = 0; i < step; ++i) {
    //double d = (1.0 - cos(PI / step * i)) * 0.5;
    // double d = 1.0 - exp(-6.0 * (double)i / (double)step);
    double x = (double)i / (double)step;
    double a = (1.0 - cos(PI * (1.0 + x))) * 0.5;
    //double b = 1.8 + 2000.0 / (double)abs(theta2_converted - KRS_NEUTRAL_ELBOW);
    double weight = 1.0 - pow(a, 2.0);
    int deg1 = round((double)theta1_start + weight * diff_theta1);
    int deg2 = round((double)theta2_start + weight * diff_theta2);
    krs.setPos(KRS_ID_ROOT, deg1);
    krs.setPos(KRS_ID_ELBOW, deg2);
    delay(delay_msec);
  }
  int deg1 = round((double)theta1_start + diff_theta1);
  int deg2 = round((double)theta2_start + diff_theta2);
  krs.setPos(KRS_ID_ROOT, deg1);
  krs.setPos(KRS_ID_ELBOW, deg2);
}

void get_disc(int rl, int bw) {
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
  int rl_get = rl;
  if (bw == WHITE) {
    rl_get ^= 1;
  }
  move_arm(DISC_SUPPLY_X, DISC_SUPPLY_Y, rl_get, KRS_SERVO_SPEED, DISC_SUPPLY_MODE);
  servo_vertical.write(VERTICAL_DEG_SUPPLY);
  delay(100);
  hold_disc_supply(rl_get);
  servo_vertical.write(VERTICAL_DEG_UP);
  delay(100);
  if (bw == WHITE) {
    move_arm(NOPOS_X, NOPOS_Y, rl_get, KRS_SERVO_SPEED, NOPOS_MODE);
    flip_disc(rl_get, BLACK);
  }
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
  /*
  for (int i = 0; i < 4; ++i) {
    get_disc(RIGHT);
    if (colors[i] == BLACK) {
      move_arm(calc_x_mm(cells[i]), calc_y_mm(cells[i]), RIGHT, KRS_SERVO_SPEED);
      lower_arm(RIGHT);
      put_disc(RIGHT, BLACK);
    } else {
      move_arm(calc_x_mm(cells[i]), calc_y_mm(cells[i]), LEFT, KRS_SERVO_SPEED);
      flip_disc(RIGHT, BLACK);
      lower_arm(LEFT);
      put_disc(LEFT, WHITE);
    }
  }
  */
  for (int i = 0; i < 4; i += 2) {
    get_disc(LEFT, WHITE);
    get_disc(RIGHT, BLACK);
    for (int j = i; j < i + 2; ++j) {
      if (colors[j] == BLACK) {
        move_arm(calc_x_mm(cells[j]), calc_y_mm(cells[j]), RIGHT, KRS_SERVO_SPEED, arm_mode[cells[j]]);
        lower_arm(RIGHT);
        put_disc(RIGHT, BLACK);
      } else {
        move_arm(calc_x_mm(cells[j]), calc_y_mm(cells[j]), LEFT, KRS_SERVO_SPEED, arm_mode[cells[j]]);
        lower_arm(LEFT);
        put_disc(LEFT, WHITE);
      }
    }
  }
}

void set_home() {
  move_arm(HOME_RIGHT_X, HOME_RIGHT_Y, RIGHT, KRS_SERVO_SPEED, HOME_RIGHT_MODE);
}

void set_camera() {
  move_arm(CAMERA_RIGHT_X, CAMERA_RIGHT_Y, RIGHT, KRS_SERVO_SPEED, CAMERA_RIGHT_MODE);
}


void loop() {
  bool player_button_pressed = false;
  while (!Serial2.available()) {
    if (!digitalRead(PLAYER_BUTTON)) {
      player_button_pressed = true;
    }
  }
  if (Serial2.available()) {
    char cmd = Serial2.read();
    if (cmd == 'p') { // put color x y
      while (Serial2.available() < 3);
      char color_char = Serial2.read();
      char x_char = Serial2.read();
      char y_char = Serial2.read();
      int color = (color_char == 'b') ? BLACK : WHITE;
      int x = x_char - '0';
      int y = y_char - '0';
      int cell = HW2 - 1 - (y * HW + x);
      if (color == BLACK) {
        get_disc(LEFT, BLACK);
      } else {
        get_disc(RIGHT, BLACK);
      }
      move_arm(calc_x_mm(cell), calc_y_mm(cell), LEFT, KRS_SERVO_SPEED, arm_mode[cell]);
      if (color == WHITE) {
        flip_disc(RIGHT, BLACK);
      }
      lower_arm(LEFT);
      put_disc(LEFT, color);
      Serial2.print('0');
    } else if (cmd == 'f') { // flip color_from x y
      while (Serial2.available() < 3);
      char color_char = Serial2.read();
      char x_char = Serial2.read();
      char y_char = Serial2.read();
      int color = (color_char == 'b') ? BLACK : WHITE;
      int x = x_char - '0';
      int y = y_char - '0';
      int cell = HW2 - 1 - (y * HW + x);
      move_arm(calc_x_mm(cell), calc_y_mm(cell), LEFT, KRS_SERVO_SPEED, arm_mode[cell]);
      lower_arm(LEFT);
      hold_disc_board(LEFT, color);
      raise_arm();
      flip_disc(LEFT, color);
      move_arm(calc_x_mm(cell), calc_y_mm(cell), RIGHT, KRS_SERVO_SPEED, arm_mode[cell]);
      lower_arm(RIGHT);
      put_disc(RIGHT, color ^ 1);
      Serial2.print('0');
    } else if (cmd == 'm') { // modify color x y diff_x_mm(+/-00) diff_y_mm(+/-00)
      while (Serial2.available() < 9);
      char color_char = Serial2.read();
      char x_char = Serial2.read();
      char y_char = Serial2.read();
      int color = (color_char == 'b') ? BLACK : WHITE;
      int x = x_char - '0';
      int y = y_char - '0';
      int cell = HW2 - 1 - (y * HW + x);
      double x_sgn = Serial2.read() == '+' ? 1 : -1;
      double diff_x_mm = Serial2.read() - '0';
      diff_x_mm = x_sgn * (diff_x_mm * 10 + Serial2.read() - '0');
      double y_sgn = Serial2.read() == '+' ? 1 : -1;
      double diff_y_mm = Serial2.read() - '0';
      diff_y_mm = y_sgn * (diff_y_mm * 10 + Serial2.read() - '0');
      move_arm(calc_x_mm(cell) + diff_x_mm, calc_y_mm(cell) + diff_y_mm, RIGHT, KRS_SERVO_SPEED, arm_mode[cell]);
      lower_arm(RIGHT);
      hold_disc_board(RIGHT, color);
      move_arm(calc_x_mm(cell), calc_y_mm(cell), RIGHT, KRS_SERVO_SPEED, arm_mode[cell]);
      put_disc(LEFT, color);
      Serial2.print('0');
    } else if (cmd == 'h') { // home
      set_home();
      Serial2.print('0');
    } else if (cmd == 'c') { // camera
      set_camera();
      Serial2.print('0');
    } else if (cmd == 'i') { // set initial board
      set_starting_board();
      Serial2.print('0');
    } else if (cmd == 'g') { // start game [color]
      while (Serial2.available() < 1);
      char color_char = Serial2.read(); // robot is black / white
      if (color_char == 'b') {
        turn_info = TURN_INFO_ROBOT;
      } else {
        turn_info = TURN_INFO_PLAYER;
      }
      Serial2.print('0');
    } else if (cmd == 'e') { // end game
      turn_info = TURN_INFO_NOT_PLAYING;
      Serial2.print('0');
    } else if (cmd == 'b') { // check player's clock button
      if (player_button_pressed) {
        Serial2.write('y');
      } else {
        bool pushed = false;
        for (int i = 0; i < 3000; ++i) {
          if (!digitalRead(PLAYER_BUTTON)) {
            Serial2.write('y');
            pushed = true;
            break;
          }
          delay(1);
        }
        if (!pushed) {
          Serial2.write('n');
        }
      }
    } else if (cmd == 's') { // set turn
      while (Serial2.available() < 1);
      char player_char = Serial2.read(); // robot / human
      if (player_char == 'r') {
        turn_info = TURN_INFO_ROBOT;
      } else if (player_char == 'h') {
        turn_info = TURN_INFO_PLAYER;
      }
      Serial2.print('0');
    } else if (cmd == 't') { // check toggle switch on clock
      if (digitalRead(TOGGLE_SWITCH)) {
        Serial2.print('h');
      } else {
        Serial2.print('r');
      }
    } else if (cmd == 'q') { // push clock
      move_arm(CLOCK_X, CLOCK_Y, RIGHT, KRS_SERVO_SPEED, CLOCK_MODE);
      servo_vertical.write(VERTICAL_DEG_CLOCK);
      delay(200);
      servo_vertical.write(VERTICAL_DEG_UP);
      Serial2.print('0');
    }
  }
}

/*
available commands:
  p: put a disc
    p[color][x][y]
    example: pb45
  f: flip a disc
    f[color_from][x][y]
    example: fw67
  m: modify a slipped disc
    m[color][x][y][diff_x_mm(+/-00)][diff_y_mm(+/-00)]
    example: mb45+05-10
  h: set arm to home position
  c: set arm to camera position
  i: set initial board
  g: start game
    g[color_robot]
    example: gw
  e: finish game
  b: check player's clock button
    returns 'y' if player pushed the button, 'n' otherwise
  s: set turn
    s[robot or human]
    example: sr (robot) / sh (human)
  t: check toggle switch on clock
    returns 'r' if robot plays black, 'h' if human plays black
  q: push clock
*/
