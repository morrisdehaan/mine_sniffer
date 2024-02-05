#include <Servo.h>
#include <ServoEasing.hpp>

// TODO: when spinning in place: first halt (<-- new one), then rotate servo's, then turn?

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

// ascii virtual keycodes
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

void computeMotorsSpeed(Direction dir);
void computeWheelAngles(Direction dir);

// all in mm
// distance from the center to the front of the car.
const int CENTER_TO_FRONT_DIST = 301;
// same but to the back side
const int CENTER_TO_BACK_DIST = 278;
// distance from the center to the left/right of the car.
const int HALF_WIDTH = 271;

// the angle of the wheels when moving in a straight line
const float STRAIGHT_WHEEL_ANGLE = 90;

Direction dir = Direction::None;

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
float turn_radius = 800;

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

/*
* Powers the left wheels in the forward direction if forward is true,
* otherwise backwards.
*/
void powerLeftWheels(bool forward) {
  if (forward) {
    // front
    analogWrite(motorW1_IN1, leftfront_wheel_speed);
    digitalWrite(motorW1_IN2, LOW);
    // middle
    analogWrite(motorW2_IN1, leftmiddle_wheel_speed);
    digitalWrite(motorW2_IN2, LOW);
    // back
    analogWrite(motorW3_IN1, leftback_wheel_speed);
    digitalWrite(motorW3_IN2, LOW);
  } else {
    // front
    analogWrite(motorW1_IN1, LOW);
    digitalWrite(motorW1_IN2, leftfront_wheel_speed);
    // middle
    digitalWrite(motorW2_IN1, LOW);
    analogWrite(motorW2_IN2, leftmiddle_wheel_speed);
    // back
    digitalWrite(motorW3_IN1, LOW);
    analogWrite(motorW3_IN2, leftback_wheel_speed);
  }
}

void powerRightWheels(bool forward) {
  if (forward) {
    // front
    analogWrite(motorW4_IN1, rightfront_wheel_speed);
    digitalWrite(motorW4_IN2, LOW);
    // middle
    analogWrite(motorW5_IN1, rightmiddle_wheel_speed);
    digitalWrite(motorW5_IN2, LOW);
    // back
    analogWrite(motorW6_IN1, rightback_wheel_speed);
    digitalWrite(motorW6_IN2, LOW);
  } else {
    // front
    analogWrite(motorW4_IN1, LOW);
    digitalWrite(motorW4_IN2, rightfront_wheel_speed);
    // middle
    digitalWrite(motorW5_IN1, LOW);
    analogWrite(motorW5_IN2, rightmiddle_wheel_speed);
    // back
    digitalWrite(motorW6_IN1, LOW);
    analogWrite(motorW6_IN2, rightback_wheel_speed);
  }
}

/*
* Turns of the left wheel motors.
*/
void unpowerLeftWheels() {
  // front
  digitalWrite(motorW1_IN1, LOW);
  digitalWrite(motorW1_IN2, LOW);
  // middle
  digitalWrite(motorW2_IN1, LOW);
  digitalWrite(motorW2_IN2, LOW);
  // back
  digitalWrite(motorW3_IN1, LOW);
  digitalWrite(motorW3_IN2, LOW);
}

void unpowerRightWheels() {
  // front
  digitalWrite(motorW4_IN1, LOW);
  digitalWrite(motorW4_IN2, LOW);
  // middle
  digitalWrite(motorW5_IN1, LOW);
  digitalWrite(motorW5_IN2, LOW);
  // back
  digitalWrite(motorW6_IN1, LOW);
  digitalWrite(motorW6_IN2, LOW);
}

void setup() {
  /*
     Use this if you need to change the frequency of the PWM signals
    TCCR4B = TCCR4B & B11111000 | B00000101;     // D6,D7,D8 PWM frequency of 30.64 Hz
    TCCR2B = TCCR2B & B11111000 | B00000111;   // D9, D10 PWM frequency of 30.64 Hz
    TCCR1B = TCCR1B & B11111000 | B00000101;   // D11, D12  PWM frequency of 30.64 Hz
    TCCR5B = TCCR5B & B11111000 | B00000101; // D4, D13 PWM frequency of 30.64 Hz
    TCCR3B = TCCR3B & B11111000 | B00000101;    // D2, D3, D5 PWM frequency of 30.64 Hz
  */
  Serial.begin(115200); // TODO: overkill
  
  // DC Motors
  unpowerLeftWheels();
  unpowerRightWheels();

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
  // process events
  if (Serial.available() > 0) {
    int key = Serial.read();

    bool change_direction;
    if (key == Keys::W && dir != Direction::Forward) {
      dir = Direction::Forward;
      change_direction = true;
    }
    else if (key == Keys::A && dir != Direction::Left) {
      dir = Direction::Left;
      change_direction = true;
    }
    else if (key == Keys::S && dir != Direction::Backward) {
      dir = Direction::Backward;
      change_direction = true;
    }
    else if (key == Keys::D && dir != Direction::Right) {
      dir = Direction::Right;
      change_direction = true;
    }
    else if (key == Keys::Q && dir != Direction::Counterclockwise) {
      dir = Direction::Counterclockwise;
      change_direction = true;
    }
    else if (key == Keys::E && dir != Direction::Clockwise) {
      dir = Direction::Clockwise;
      change_direction = true;
    }
    else if (key == Keys::Space && dir != Direction::None) {
      dir = Direction::None;
      change_direction = true;
    }
    else {
      change_direction = false;
    }

    if (change_direction) {
      computeWheelAngles(dir);

      // rotate wheels
      leftfront_servo.startEaseTo(leftfront_wheel_angle);
      leftback_servo.startEaseTo(leftback_wheel_angle);
      rightfront_servo.startEaseTo(rightfront_wheel_angle);
      rightback_servo.startEaseTo(rightback_wheel_angle);

      computeMotorsSpeed(dir);  
      powerWheelMotors(dir);
    }
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

/*
* Computes the necessary absolute speed for each wheel such that the vehicle
* moves into the desired direction.
*/
void computeMotorsSpeed(Direction dir) {
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
    float ifw_circum = circumference(distance(turn_radius - HALF_WIDTH, CENTER_TO_FRONT_DIST));
    float imw_circum = circumference(turn_radius - HALF_WIDTH);
    float ibw_circum = circumference(distance(turn_radius - HALF_WIDTH, CENTER_TO_BACK_DIST));
    // outer wheels (front, middle and back)
    float ofw_circum = circumference(distance(turn_radius + HALF_WIDTH, CENTER_TO_FRONT_DIST));
    float omw_circum = circumference(turn_radius + HALF_WIDTH);
    float obw_circum = circumference(distance(turn_radius + HALF_WIDTH, CENTER_TO_BACK_DIST));

    // the outer middle wheel is always the farthest away from the turning center, so we set it equal to the cruise speed
    //  such that no wheel's speed exceeds the cruise speed
    float outermiddle_wheel_speed = CRUISE_SPEED;

    // Because we want to turn, the inner wheels (relative to the turning center) should go slower than the outer ones,
    //  i.e. we want each wheel to travel the circumference of the circle with the turning center in the middle
    //  and the wheel on it in the same time. When turning, every wheel has a different circumference, so
    //  we compute a different speed for each wheel, all relative to the fastest moving wheel (the outer middle wheel).
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

/*
* Calculate wheel angles for the given movement direction
* based on Ackerman steering geometry.
*
* It's assumed that all leftside servos spin clockwise.
*/
void computeWheelAngles(Direction dir) {
  if (dir == Direction::Left || dir == Direction::Right || dir == Direction::Counterclockwise || dir == Direction::Clockwise) {
    // do the necessary Ackerman trigonometry, i.e.
    //  rotate the wheels such that they align a circle
    //  around the turning center
    float turning_radius;
    if (dir == Direction::Left) turning_radius = turn_radius;
    else if (dir == Direction::Right) turning_radius = -turn_radius;
    else turning_radius = 0.0;

    // turn clockwise
    leftfront_wheel_angle = STRAIGHT_WHEEL_ANGLE - atan((double)CENTER_TO_FRONT_DIST / (double)(turning_radius - HALF_WIDTH)) * 180.0 / PI;
    leftback_wheel_angle = STRAIGHT_WHEEL_ANGLE + atan((double)CENTER_TO_BACK_DIST / (double)(turning_radius - HALF_WIDTH)) * 180.0 / PI;

    rightfront_wheel_angle = STRAIGHT_WHEEL_ANGLE - atan((double)CENTER_TO_FRONT_DIST / (double)(turning_radius + HALF_WIDTH)) * 180.0 / PI;
    rightback_wheel_angle = STRAIGHT_WHEEL_ANGLE + atan((double)CENTER_TO_BACK_DIST / (double)(turning_radius + HALF_WIDTH)) * 180.0 / PI;
  } else {
    leftfront_wheel_angle = STRAIGHT_WHEEL_ANGLE;
    leftback_wheel_angle = STRAIGHT_WHEEL_ANGLE;
    rightfront_wheel_angle = STRAIGHT_WHEEL_ANGLE;
    rightback_wheel_angle = STRAIGHT_WHEEL_ANGLE;
  }
}

/*
* Powers the motors accordingly to the computed speeds.
* This function will stall time until the servos finish rotating if spinning in place.
*/
void powerWheelMotors(Direction dir) {
  // power DC motors
  if (dir == Direction::Forward || dir == Direction::Right || dir == Direction::Left) {
    powerLeftWheels(true);
    powerRightWheels(true);
  }  
  else if (dir == Direction::Backward) {
    powerLeftWheels(false);
    powerRightWheels(false);
  }
  else if (dir == Direction::Counterclockwise || dir == Direction::Clockwise) {
    // wait until servo's have finished rotating if turning in place
    if (areWheelsRotating()) {
      unpowerLeftWheels();
      unpowerRightWheels();
      while (areWheelsRotating()) delay(50); 
    }

    bool spin_dir = dir == Direction::Counterclockwise;

    // TODO: this is doubled??
    powerLeftWheels(!spin_dir);
    powerRightWheels(spin_dir);
    powerLeftWheels(!spin_dir);
    powerRightWheels(spin_dir);
  }
  else {
    unpowerLeftWheels();
    unpowerRightWheels();
  }
}

/*
* Returns `true` if any of the wheels operated by the servos are still rotating.
*/
bool areWheelsRotating() {
  return leftfront_servo.isMoving() || leftback_servo.isMoving() || rightfront_servo.isMoving() || rightback_servo.isMoving();
}