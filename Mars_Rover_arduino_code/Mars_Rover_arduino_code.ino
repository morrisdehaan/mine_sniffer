#include <Servo.h>
#include <ServoEasing.hpp>

/*
 *    Mine Sniffer Arduino Code
 * 
 *   Libraries:
 *   ServoEasing: https://github.com/ArminJo/ServoEasing
 */

// motor input pins
#define motorW1_IN1 6
#define motorW1_IN2 7 //PWM
#define motorW2_IN1 4
#define motorW2_IN2 5 //PWM
#define motorW3_IN1 2
#define motorW3_IN2 3 //PWM
#define motorW4_IN1 13
#define motorW4_IN2 10 //PWM
#define motorW5_IN1 8
#define motorW5_IN2 9 //PWM
#define motorW6_IN1 11
#define motorW6_IN2 12 //PWM

#define motorW1_SERVO 22
#define motorW3_SERVO 23
#define motorW4_SERVO 24
#define motorW6_SERVO 25

/*
* Moves the left-front wheel motor in the forward direction if move_forward is true,
* otherwise the moter moves backwards.
*/
void power_leftfront_wheel(uint8_t power, bool move_forward) {
  if (move_forward) {
    analogWrite(motorW1_IN1, power);
    digitalWrite(motorW1_IN2, LOW);
  } else {
    analogWrite(motorW1_IN1, LOW);
    digitalWrite(motorW1_IN2, power);
  }
}

void power_leftmiddle_wheel(uint8_t power, bool move_forward) {
  if (move_forward) {
    analogWrite(motorW2_IN1, power);
    digitalWrite(motorW2_IN2, LOW);
  } else {
    digitalWrite(motorW2_IN1, LOW);
    analogWrite(motorW2_IN2, power);
  }
}

void power_leftback_wheel(uint8_t power, bool move_forward) {
  if (move_forward) {
    analogWrite(motorW3_IN1, power);
    digitalWrite(motorW3_IN2, LOW);
  } else {
    digitalWrite(motorW3_IN1, LOW);
    analogWrite(motorW3_IN2, power);
  }
}

void power_rightfront_wheel(uint8_t power, bool move_forward) {
  // NOTE: the motors on the right side move in the opposite direction
  if (move_forward) {
    digitalWrite(motorW4_IN1, LOW);
    analogWrite(motorW4_IN2, power);
  } else {
    analogWrite(motorW4_IN1, power);
    digitalWrite(motorW4_IN2, LOW);
  }  
}

void power_rightmiddle_wheel(uint8_t power, bool move_forward) {
  if (move_forward) {
    digitalWrite(motorW5_IN1, LOW);
    analogWrite(motorW5_IN2, power);
  } else {
    analogWrite(motorW5_IN1, power);
    digitalWrite(motorW5_IN2, LOW);
  }  
}

void power_rightback_wheel(uint8_t power, bool move_forward) {
  if (move_forward) {
    digitalWrite(motorW6_IN1, LOW);
    analogWrite(motorW6_IN2, power);
  } else {
    analogWrite(motorW6_IN1, power);
    digitalWrite(motorW6_IN2, LOW);
  }  
}

/*
* Turns of the left-front motor.
*/
void unpower_leftfront_wheel() {
  digitalWrite(motorW1_IN1, LOW);
  digitalWrite(motorW1_IN2, LOW);
}

void unpower_leftmiddle_wheel() {
  digitalWrite(motorW2_IN1, LOW);
  digitalWrite(motorW2_IN2, LOW);
}

void unpower_leftback_wheel() {
  digitalWrite(motorW3_IN1, LOW);
  digitalWrite(motorW3_IN2, LOW);
}

void unpower_rightfront_wheel() {
  digitalWrite(motorW4_IN1, LOW);
  digitalWrite(motorW4_IN2, LOW);
}

void unpower_rightmiddle_wheel() {
  digitalWrite(motorW5_IN1, LOW);
  digitalWrite(motorW5_IN2, LOW);
}

void unpower_rightback_wheel() {
  digitalWrite(motorW6_IN1, LOW);
  digitalWrite(motorW6_IN2, LOW);
}

enum Keys {
  Space = 32,
  A = 97,
  E = 101,
  D = 100,
  Q = 113,
  S = 115,
  W = 119
};

enum Direction {
	Forward, Backward, Left, Right, Clockwise, Counterclockwise, None
};

void calculateMotorsSpeed(Direction dir);
void calculateWheelAngles(Direction dir);

Direction dir = Direction::None;

// all in mm
// distance from the center to the front of the car
const int CENTER_TO_FRONT_DIST = 301;
// same but to the back side
const int CENTER_TO_BACK_DIST = 278;
// distance from the center to the left of the car (same as to right side)
const int CENTER_TO_LEFT_DIST = 271; // TODO: rename

// the angle of the wheels when moving in a straight line
const float STRAIGHT_WHEEL_ANGLE = 90;

// rover speed when moving
int CRUISE_SPEED = 100;
// rover speed when rotating in place
int ROTATION_SPEED = 50;

// rotations per minute
int SERVO_ROTATION_SPEED = 550;
// rover speed
float speed = 0;

// TODO: update in code
// in mm
float turning_radius = 1600;

// the servos rotate the front and back wheels
ServoEasing leftfront_servo;
ServoEasing leftback_servo;
ServoEasing rightfront_servo;
ServoEasing rightback_servo;

float leftfront_wheel_angle = 0;
float leftback_wheel_angle = 0;
float rightfront_wheel_angle = 0;
float rightback_wheel_angle = 0;

float leftfront_wheel_speed = 0;
float leftmiddle_wheel_speed = 0;
float leftback_wheel_speed = 0;
float rightfront_wheel_speed = 0;
float rightmiddle_wheel_speed = 0;
float rightback_wheel_speed = 0;

void setup() {
  /*
     Use this if you need to change the frequency of the PWM signals
    TCCR4B = TCCR4B & B11111000 | B00000101;     // D6,D7,D8 PWM frequency of 30.64 Hz
    TCCR2B = TCCR2B & B11111000 | B00000111;   // D9, D10 PWM frequency of 30.64 Hz
    TCCR1B = TCCR1B & B11111000 | B00000101;   // D11, D12  PWM frequency of 30.64 Hz
    TCCR5B = TCCR5B & B11111000 | B00000101; // D4, D13 PWM frequency of 30.64 Hz
    TCCR3B = TCCR3B & B11111000 | B00000101;    // D2, D3, D5 PWM frequency of 30.64 Hz
  */
  Serial.begin(115200);
  
  // DC Motors
  unpower_leftfront_wheel();
  unpower_leftmiddle_wheel();
  unpower_leftback_wheel();
  unpower_rightfront_wheel();
  unpower_rightmiddle_wheel();
  unpower_rightback_wheel();

  // servos
  leftfront_servo.attach(motorW1_SERVO);
  leftback_servo.attach(motorW3_SERVO);
  rightfront_servo.attach(motorW4_SERVO);
  rightback_servo.attach(motorW6_SERVO);

  leftfront_servo.setSpeed(SERVO_ROTATION_SPEED);
  leftback_servo.setSpeed(SERVO_ROTATION_SPEED);
  rightfront_servo.setSpeed(SERVO_ROTATION_SPEED);
  rightback_servo.setSpeed(SERVO_ROTATION_SPEED);

  // set correct angle
  leftfront_servo.write(STRAIGHT_WHEEL_ANGLE);
  leftback_servo.write(STRAIGHT_WHEEL_ANGLE);
  rightfront_servo.write(STRAIGHT_WHEEL_ANGLE);
  rightback_servo.write(STRAIGHT_WHEEL_ANGLE);
  
  dir = Direction::None;
}

void loop() {
  // process key input
  if (Serial.available() > 0) {
    int key = Serial.read();

    switch (key) {
      case Keys::W:
        dir = Direction::Forward;
        break;
      case Keys::A:
        dir = Direction::Left;
        break;
      case Keys::S:
        dir = Direction::Backward;
        break;
      case Keys::D:
        dir = Direction::Right;
        break;
      case Keys::Q:
        dir = Direction::Counterclockwise;
        break;
      case Keys::E:
        dir = Direction::Clockwise;
        break;
      case Keys::Space:
        dir = Direction::None;
    }
  }

  calculateMotorsSpeed(dir);
  calculateWheelAngles(dir);

  // rotate wheels
  leftfront_servo.startEaseTo(leftfront_wheel_angle);
  leftback_servo.startEaseTo(leftback_wheel_angle);
  rightfront_servo.startEaseTo(rightfront_wheel_angle);
  rightback_servo.startEaseTo(rightback_wheel_angle);

  // power DC motors
  if (dir == Direction::Right) {
    power_leftfront_wheel(leftfront_wheel_speed, true);
    power_leftmiddle_wheel(leftmiddle_wheel_speed, true);
    power_leftback_wheel(leftback_wheel_speed, true);
    power_rightfront_wheel(rightfront_wheel_speed, true);
    power_rightmiddle_wheel(rightmiddle_wheel_speed, true);
    power_rightback_wheel(rightback_wheel_speed, true);
  }  
  else if (dir == Direction::Left) {
    power_leftfront_wheel(leftfront_wheel_speed, true);
    power_leftmiddle_wheel(leftmiddle_wheel_speed, true);
    power_leftback_wheel(leftback_wheel_speed, true);
    power_rightfront_wheel(rightfront_wheel_speed, true);
    power_rightmiddle_wheel(rightmiddle_wheel_speed, true);
    power_rightback_wheel(rightback_wheel_speed, true);
  }  
	else if (dir == Direction::Forward) {
    power_leftfront_wheel(leftfront_wheel_speed, true);
    power_leftmiddle_wheel(leftmiddle_wheel_speed, true);
    power_leftback_wheel(leftback_wheel_speed, true);
    power_rightfront_wheel(rightfront_wheel_speed, true);
    power_rightmiddle_wheel(rightmiddle_wheel_speed, true);
    power_rightback_wheel(rightback_wheel_speed, true);
  }  
  else if (dir == Direction::Backward) {
    power_leftfront_wheel(leftfront_wheel_speed, false);
    power_leftmiddle_wheel(leftmiddle_wheel_speed, false);
    power_leftback_wheel(leftback_wheel_speed, false);
    power_rightfront_wheel(rightfront_wheel_speed, false);
    power_rightmiddle_wheel(rightmiddle_wheel_speed, false);
    power_rightback_wheel(rightback_wheel_speed, false);
  }
  else if (dir == Direction::Counterclockwise || dir == Direction::Clockwise) {
    // wait for the servos to stop moving
    if (!leftfront_servo.isMoving() && !leftback_servo.isMoving() &&
        !rightfront_servo.isMoving() && !rightback_servo.isMoving())
    {
      bool spin_dir = dir == Direction::Counterclockwise;

      // move all wheels forward if moving counterclockwise,
      //  otherwise move backwards
      power_leftfront_wheel(speed, spin_dir);
      power_leftmiddle_wheel(speed, spin_dir);
      power_leftback_wheel(speed, spin_dir);
      power_rightfront_wheel(speed, spin_dir);
      power_rightmiddle_wheel(speed, spin_dir);
      power_rightback_wheel(speed, spin_dir);
    }
  }
  else {
    unpower_leftfront_wheel();
    unpower_leftmiddle_wheel();
    unpower_leftback_wheel();
    unpower_rightfront_wheel();
    unpower_rightmiddle_wheel();
    unpower_rightback_wheel();
  }
}

/*
* Computes circumference of a circle with the given radius.
*/
float circumference(float radius) {
  return 2.0 * PI * radius;
}

float distance(float dx, float dy) {
  return sqrt(dx * dx + dy * dy);
}

void calculateMotorsSpeed(Direction dir) {
  if (dir == Direction::Forward || dir == Direction::Backward) {
    leftfront_wheel_speed = leftmiddle_wheel_speed = leftback_wheel_speed = 
      rightfront_wheel_speed = rightmiddle_wheel_speed = rightback_wheel_speed = CRUISE_SPEED;
  }
  else if (dir == Direction::Counterclockwise || dir == Direction::Clockwise) {
    leftfront_wheel_speed = leftmiddle_wheel_speed = leftback_wheel_speed = 
      rightfront_wheel_speed = rightmiddle_wheel_speed = rightback_wheel_speed = ROTATION_SPEED;
  }
  else if (dir == Direction::Left || dir == Direction::Right) {
    // inner-front wheel circumference of the circle around the turning center, repeat for all wheels
    float ifw_circum = circumference(distance(turning_radius - CENTER_TO_LEFT_DIST, CENTER_TO_FRONT_DIST));
    float imw_circum = circumference(turning_radius - CENTER_TO_LEFT_DIST);
    float ibw_circum = circumference(distance(turning_radius - CENTER_TO_LEFT_DIST, CENTER_TO_BACK_DIST));
    // outer wheels (front, middle and back)
    float ofw_circum = circumference(distance(turning_radius + CENTER_TO_LEFT_DIST, CENTER_TO_FRONT_DIST));
    float omw_circum = circumference(turning_radius + CENTER_TO_LEFT_DIST);
    float obw_circum = circumference(distance(turning_radius + CENTER_TO_LEFT_DIST, CENTER_TO_BACK_DIST));

    // the outer middle wheel is always the farthest away from the turning center, so we set it equal to the cruise speed
    float outermiddle_wheel_speed = CRUISE_SPEED;

    // We want every wheel to travel the same distance in the same time, i.e. to travel the circumference of their circle
    //  with the turning center in the middle in the same time. When turning, every wheel has a different speed, so
    //  we compute a different speed for each wheel, all relative to the fastest moving wheel (the outer middle wheel,
    //  because it is farthest away from the turning center).
    float innerfront_wheel_speed = ifw_circum / omw_circum * outermiddle_wheel_speed;
    float innermiddle_wheel_speed = imw_circum / omw_circum * outermiddle_wheel_speed;
    float innerback_wheel_speed = ibw_circum / omw_circum * outermiddle_wheel_speed;
    
    float outerfront_wheel_speed = ofw_circum / omw_circum * outermiddle_wheel_speed;
    float outerback_wheel_speed = obw_circum / omw_circum * outermiddle_wheel_speed;

    if (dir == Direction::Left) {
      leftfront_wheel_speed = innerfront_wheel_speed;
      leftmiddle_wheel_speed = innermiddle_wheel_speed;
      leftback_wheel_speed = innerback_wheel_speed;
      
      rightfront_wheel_speed = outerfront_wheel_speed;
      rightmiddle_wheel_speed = outermiddle_wheel_speed;
      rightback_wheel_speed = outerback_wheel_speed;
    } else if (dir == Direction::Right) {
      leftfront_wheel_speed = outerfront_wheel_speed;
      leftmiddle_wheel_speed = outermiddle_wheel_speed;
      leftback_wheel_speed = outerback_wheel_speed;
      
      rightfront_wheel_speed = innerfront_wheel_speed;
      rightmiddle_wheel_speed = innermiddle_wheel_speed;
      rightback_wheel_speed = innerback_wheel_speed;
    }
  }
}

// TODO: cache or make constants of atan computations
/*
* Calculate wheel angles for the given movement direction
* based on Ackerman steering geometry.
* Rotations are assumed to go counterclockwise.
*/
void calculateWheelAngles(Direction dir) {
  if (dir == Direction::Left || dir == Direction::Right) {
    // do the necessary Ackerman trigonometry, i.e.
    //  rotate the wheels such that they align a circle
    //  around the turning center
    float innerfront_wheel_angle = STRAIGHT_WHEEL_ANGLE + atan(CENTER_TO_FRONT_DIST / (turning_radius - CENTER_TO_LEFT_DIST)) * 180 / PI;
    float innerback_wheel_angle = STRAIGHT_WHEEL_ANGLE - atan(CENTER_TO_BACK_DIST / (turning_radius - CENTER_TO_LEFT_DIST)) * 180 / PI;

    float outerfront_wheel_angle = STRAIGHT_WHEEL_ANGLE + atan(CENTER_TO_FRONT_DIST / (turning_radius + CENTER_TO_LEFT_DIST)) * 180 / PI;
    float outerback_wheel_angle = STRAIGHT_WHEEL_ANGLE - atan(CENTER_TO_BACK_DIST / (turning_radius + CENTER_TO_LEFT_DIST)) * 180 / PI;

    if (dir == Direction::Left) {
      leftfront_wheel_angle = innerfront_wheel_angle;
      leftback_wheel_angle = innerback_wheel_angle;
      
      rightfront_wheel_angle = outerfront_wheel_angle;
      rightback_wheel_angle = outerback_wheel_angle;
    } else if (dir == Direction::Right) {
      leftfront_wheel_angle = outerfront_wheel_angle;
      leftback_wheel_angle = outerback_wheel_angle;
      
      rightfront_wheel_angle = innerfront_wheel_angle;
      rightback_wheel_angle = innerback_wheel_angle;
    }
  } else if (dir == Direction::Counterclockwise || dir == Direction::Clockwise) {
    // rotate the wheels such that they align the circle around the center of the car
    leftfront_wheel_angle = STRAIGHT_WHEEL_ANGLE - atan(CENTER_TO_LEFT_DIST / CENTER_TO_FRONT_DIST) * 180 / PI;
    leftback_wheel_angle = STRAIGHT_WHEEL_ANGLE + atan(CENTER_TO_LEFT_DIST / CENTER_TO_BACK_DIST) * 180 / PI;

    rightfront_wheel_angle = STRAIGHT_WHEEL_ANGLE + atan(CENTER_TO_LEFT_DIST / CENTER_TO_FRONT_DIST) * 180 / PI - 180;
    rightback_wheel_angle = STRAIGHT_WHEEL_ANGLE - atan(CENTER_TO_LEFT_DIST / CENTER_TO_BACK_DIST) * 180 / PI + 180;
  }
  else {
    leftfront_wheel_angle = STRAIGHT_WHEEL_ANGLE;
    leftback_wheel_angle = STRAIGHT_WHEEL_ANGLE;
    rightfront_wheel_angle = STRAIGHT_WHEEL_ANGLE;
    rightback_wheel_angle = STRAIGHT_WHEEL_ANGLE;
  }
}



