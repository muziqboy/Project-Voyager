/*
KOD till kursen Mj1104 - intro till energiteknik
Vidar Höjer
*/

/* HELPFULL ABOUT ARDUINO
    digitalRead() works on all pins. It will just round the analog value received and present it to you. If analogRead(A0) is greater than or equal to 512, digitalRead(A0) will be 1, else 0.
    digitalWrite() works on all pins, with allowed parameter 0 or 1. digitalWrite(A0,0) is the same as analogWrite(A0,0), and digitalWrite(A0,1) is the same as analogWrite(A0,255)
    analogRead() works only on analog pins. It can take any value between 0 and 1023.
    analogWrite() works on all analog pins and all digital PWM pins. You can supply it any value between 0 and 255.
*/



#include <Servo.h>

Servo servomotor;  // create servo object to control a servo

// int pos = 0;    // variable to store the servo position

// Attach the Servo variable to a pin. Note that in Arduino 0016 and earlier, the Servo library supports only servos on only two pins: 9 and 10.
// Possible bug!!!
const int servo_pin = 1;


const int motor_A_pin_1 = 2;
const int motor_A_pin_2 = 3;

const int motor_B_pin_1 = 4;
const int motor_B_pin_2 = 5;

const int IR_left_pin  = 6;
const int IR_right_pin = 7;

const int EN_A_pin  = 9;  // can be analog pins
const int EN_B_pin = 10;  // can be analog pins

const int US_echo_pin = 11;
const int US_trig_pin = 12;

/// GLOBALS
int turning_radius = 10;
int tol = 5;          // tolerance of turning radius i.e (5-15 is OK)
/////////// Defined functions /////////

void irTest();
void motorTest();
void stopMotors();
bool runMotors(int time_to_run = 500, int speed_motor_A = 255, int speed_motor_B = 255);
int distanceUS(int pos, int num_vals = 2);
int closestObject(int increment = 5);
void faceObject(int N = 10);
bool checkIRsensor(int sensorPin);
int turnXdegrees(bool turn_right = true, bool feedback90degree = true, int max_iterations = 100);


void setup() {
  /*
  Sketch:
  Sweep servo to scan with US sensor,
  find closest target,
  Pass to the right of object while staying equidistant from it,
  continue until on other side of object,
  reverse from object untill in between parking lines (check with IR sensors)
  find ramp and acccellerate up to finish line.
  */
  // DC_MOTORER
  pinMode(motor_A_pin_1, OUTPUT);
  pinMode(motor_A_pin_2, OUTPUT);
  pinMode(motor_B_pin_1, OUTPUT);
  pinMode(motor_B_pin_2, OUTPUT);

  pinMode(EN_A_pin, OUTPUT);
  pinMode(EN_B_pin, OUTPUT);
  // IR
  pinMode(IR_left_pin, INPUT);
  pinMode(IR_right_pin, INPUT);
  // SERVO
  servomotor.attach(servo_pin);
  // ULTRASONIC
  pinMode(US_trig_pin, OUTPUT);
  digitalWrite(US_trig_pin, LOW); // default off
  pinMode(US_echo_pin, INPUT);

  Serial.begin(9600); // opens serial window for printing values
}

void loop(){
  /*
  MAIN Loop
  */
  bool runTESTS = true; // turn to false when code has been tested
  // TESTFUNCTIONS
  if (runTESTS) {
    irTest();
    motorTest();
  }
  else {
    //
    // int min_distance = 10; // distance from obstacle
    // int tol = 5;           // tolerance or half car width
    // curve constants
    int time_step   = 100;
    int speed_motor = 200;
    int diff_speed  = 50;
    // start on line
          ////////      ////////      ////////      ////////
    int object_direction = closestObject();
    int object_distance = distanceUS(object_direction);
    Serial.println("Moving towards obstacle...");
    while (object_distance > turning_radius) {
      // drive forward until closer to obstacle
      runMotors();
      // update variables
      object_direction = closestObject();
      object_distance =  distanceUS(object_direction);
      Serial.println("IR signals");
      Serial.print(checkIRsensor(IR_left_pin));
      Serial.print(checkIRsensor(IR_right_pin));
    }
    // HALTS MOTION BEFORE NEXT  STEP
    stopMotors();
      ////////      ////////      ////////      ////////
    Serial.println("Turning 90 degrees right...");
    // when 10cm from first obstacle turn 90 degrees right.
    int turn_iterations = turnXdegrees(true, true); // right_turn 90degree
    Serial.print("Turn iterations: ");
    Serial.println(turn_iterations);
    // first 90 degree turn gives us data for future reference
          ////////      ////////      ////////      ////////
    // equidistant turning around obstacle
    bool left_IR = checkIRsensor(IR_left_pin);
    bool right_IR = checkIRsensor(IR_right_pin);
    Serial.println("Turning around obstacle, ADD stop to program when");
    Serial.println("car has passed obstacle note the time_step! (3 seconds pause!)");
    delay(3000);
    int time_counter = 0; // FIND OUT HOW MANY ITERATIONS to go half circle
    while (left_IR && right_IR) {
      ////////
      // ASSSUMPTION MOTOR A IS INNER (left)
      ///////
      object_direction = closestObject();
      object_distance = distanceUS(object_direction);

      if (object_distance < turning_radius - tol) {
        // too close to object
        Serial.println("Too close");
        //       (time_to_run,    speed_motor_A,           speed_motor_B)
        runMotors(time_step, speed_motor+diff_speed, speed_motor-diff_speed);
      }
      else if (object_distance > turning_radius - tol) {
        // too far from object
        Serial.println("Too far");
        runMotors(time_step, speed_motor-diff_speed, speed_motor+diff_speed);
      }
      else {
        // good range
        Serial.println("OK Range");
        runMotors(time_step, speed_motor, speed_motor);
      }
      left_IR = checkIRsensor(IR_left_pin);
      right_IR = checkIRsensor(IR_right_pin);
      time_counter += time_step;
      Serial.print("Timestep: ");
      Serial.println(time_counter);
      if (!left_IR || !right_IR){
        // edge found
        Serial.println("ERROR IR SENSORS ARE TRIGGERED!");
        stopMotors();
        break;
      }
    }
    ////////      ////////      ////////      ////////
  // Turn blindly 90 degrees and correct with US sensor towards target_wall
  stopMotors(); // run again if not stopped
  Serial.println("Turning 90 degrees right...");
  int correction = 0; // for calibrating better curve
  turnXdegrees(true, false, turn_iterations+correction); // NO US sensor turns a set amount
    ////////      ////////      ////////      ////////
  // FACE TOWARDS target_wall
  Serial.println("looking for obstacle... (direction ~ 90 if good turn)");
  faceObject();
  //////   //////   //////   //////
  // Move closer to target_wall,
  // ASSUMPTION  //////   //////   //////   ////// //////   //////   //////   //////
  // parking space ~100cm from wall???
  while (object_distance > 100) {
    runMotors(100);
    object_direction = closestObject();
    object_distance = distanceUS(object_direction);
  }
  stopMotors();
  Serial.println("PARKED!! (restart in...)");
  for (int x = 5; x > 0; x--){
    Serial.println(x);
    delay(1000);
  }
  //////   ////// //////   //////   //////   //////
  // Final steps,
  Serial.println("Approaching Final target!");
  bool no_IR_trig = runMotors(100);
  while (no_IR_trig){
    no_IR_trig = runMotors(100);
    object_direction = closestObject();
    object_distance = distanceUS(object_direction);
    Serial.print(object_distance);
    Serial.print(" cm \n");
  }
  // FINISHED!!
  Serial.println("TASK Finished car paused for 10 seconds!");
  for (int x = 10; x > 0; x--){
    Serial.println(x);
    delay(1000);
  }
  }
}

/////////// FUNCTIONS //////////////
void irTest() {
  /*
  Function to test IR sensor, keep outputting values untill both sensors are calibrated, Formatted for reading with serial window
  */
  Serial.println("TESTING IR Sensors");
  int i;
  Serial.print("Left Sensor    |    Right Sensor");
  for (i = 0;i < 100; i++) {
    int sensor_left = digitalRead(IR_left_pin);
    int sensor_right = digitalRead(IR_right_pin);
    Serial.print(sensor_left);
    Serial.print("    |    ");
    Serial.println(sensor_right);
    delay(500); // HALF Second between outputs
  }
}

void motorTest(){
  /*
  Testfunction to ensure motor directions are correct
  */

  // Controlling speed (0 = off and 255 = max speed):
  // AWARE! ANALOGsignal
  Serial.println("TESTING MOTORS");
  Serial.println("Make sure large area availible!");
  analogWrite(EN_A_pin, 100); //ENA pin
  analogWrite(EN_B_pin, 100); //ENB pin
  Serial.println("Motor A only");
  digitalWrite(motor_B_pin_1, LOW);
  digitalWrite(motor_B_pin_2, LOW); // stop
  //Controlling spin direction of motors
  // (HIGH,HIGH) or (LOW,LOW) -> STOP
  // (HIGH,LOW) or (LOW,HIGH) -> for CW or CCW
  digitalWrite(motor_A_pin_1, HIGH);
  digitalWrite(motor_A_pin_2, LOW);
  Serial.println("CLOCKWISE 2 sec (motor_A_pin_1 = HIGH), (motor_A_pin_2 = LOW)");
  delay(2000);

  digitalWrite(motor_A_pin_1, LOW);
  digitalWrite(motor_A_pin_2, HIGH);
  Serial.println("Counter CLOCKWISE 2 sec (motor_A_pin_1 = LOW), (motor_A_pin_2 = HIGH)");
  delay(2000);

  Serial.println("MOTOR B only!");
  // digitalWrite(motor_A_pin_1, LOW);
  digitalWrite(motor_A_pin_2, LOW); // stop

  digitalWrite(motor_B_pin_1, LOW);
  digitalWrite(motor_B_pin_2, HIGH);
  Serial.println("CLOCKWISE 2 sec");
  delay(2000);

  digitalWrite(motor_B_pin_1, HIGH);
  digitalWrite(motor_B_pin_2, LOW);
  Serial.println("Counter CLOCKWISE 2 sec");
  delay(2000);
  digitalWrite(motor_B_pin_1, LOW); // stop

  Serial.println("Forward");
  digitalWrite(motor_A_pin_1, LOW);
  digitalWrite(motor_A_pin_2, HIGH);
  digitalWrite(motor_B_pin_1, HIGH);
  digitalWrite(motor_B_pin_2, LOW);
  delay(2000);
  Serial.println("Backward");
  digitalWrite(motor_A_pin_1, HIGH);
  digitalWrite(motor_A_pin_2, LOW);
  digitalWrite(motor_B_pin_1, LOW);
  digitalWrite(motor_B_pin_2, HIGH);
  delay(2000);
  digitalWrite(motor_B_pin_2, LOW); // stop
  digitalWrite(motor_A_pin_1, LOW); // stop
  /////////////////////////
  Serial.println("TESTING SERVO");
  servomotor.write(0);
  Serial.println(">Left");
  delay(2000);
  servomotor.write(90);
  Serial.println(">MIDDLE");
  delay(2000);
  servomotor.write(180);
  Serial.println(">RIGHT");
  delay(2000);
}

void stopMotors(){
  // FAST STOP to motors incase of danger!
  analogWrite(EN_A_pin, 0);
  analogWrite(EN_B_pin, 0);
  digitalWrite(motor_A_pin_1, LOW);
  digitalWrite(motor_A_pin_2, LOW);
  digitalWrite(motor_B_pin_1, LOW);
  digitalWrite(motor_B_pin_2, LOW);
}

bool runMotors(int time_to_run = 500, int speed_motor_A = 255, int speed_motor_B = 255){
  /*
  runs motors for a prescirbed time
  if no speed incicated -> both engines max speed
  In  -> Time in mS (default 1000); motorspeeds (0-255)
  Out -> None
  */
  int stepsize = 10;
  analogWrite(EN_A_pin, speed_motor_A);
  analogWrite(EN_B_pin, speed_motor_B);
  for (time_to_run; time_to_run > 0; time_to_run-=stepsize){
    // MAKE SURE DIRECTIONS ARE CORRECT
    digitalWrite(motor_A_pin_1, HIGH);
    digitalWrite(motor_A_pin_2, LOW);
    digitalWrite(motor_B_pin_1, LOW);
    digitalWrite(motor_B_pin_2, HIGH);
    delay(stepsize);
    if (!checkIRsensor(IR_left_pin) ||(!checkIRsensor(IR_right_pin))){
      Serial.println("ERROR, Unexpected IR-trigger!");
      return false;
    }
  }
  return true;
}

int distanceUS(int pos, int num_vals = 2){
  /*
  reads from the US sensors, average of num_vals values,
  default average of 2 readings
  depends on quality of measurements
  IN  -> angle of servo motor to read from
  OUT -> value in cm
  */
  int value = 0;
  int i;
  for (i = 0; i < num_vals; i++){
    servomotor.write(pos);
    digitalWrite(US_trig_pin, HIGH);
    delayMicroseconds(10);
    digitalWrite(US_trig_pin, LOW);
    // Reads time between two HIGH pulses
    value += pulseIn(US_echo_pin, HIGH) / 58;
    delayMicroseconds(15);
  }
  value = value/num_vals;
  Serial.println("> US sensor working < ");
  Serial.print("Distance in cm : ");
  Serial.println(value);
  return value;
}

int closestObject(int increment = 5){
  /*
   Sweeps servo to find closest obstacle with US sensors
  ALWAYS SWEEPs FULL SPECTRUM,
  Decrease stepsize to increase resulution of sweep
  In  -> default argument for sweep resulution
  Out -> Direction relative to car
  */
  int min_val = 100;  //
  int min_direction;
  int pos;  // start position
  for (pos = 0;pos <= 180; pos += increment){
    int temp_val = distanceUS(pos);
    if (temp_val < min_val){
      min_val = temp_val;
      min_direction = pos;
    }
  }
  return min_direction;
}

void faceObject(int N = 10){
  /*
  Turn so closest object is straight infront of car
  IN -> max step iterations
  OUT -> None
  */
  int object_direction = closestObject();
  int object_distance = distanceUS(object_direction);
   // 10 iterations to try and fix turn
  Serial.print("> Direction: ");
  Serial.print(object_direction);
  Serial.print(" | Distance (cm): ");
  Serial.println(object_distance);
  Serial.println("(3 seconds pause)");
  delay(3000);
  while (abs(object_direction - 90) > 2 && N > 0){
    Serial.println("correcting");
    if (object_direction < 88) {
      // Target is to the left
      turnXdegrees(false, false, N);
    }
    else if (object_direction > 92){
      // Target is to the right
      turnXdegrees(true, false, N);
    }
    N--;
  }
}

bool checkIRsensor(int sensorPin){
  /*
  !!! FIRST CALIBRATE SENSOR WITH ONBOARD VARIABLE RESISTOR !!!
  function tests the sensor until it detects different surfaces at
  correct height
  IN  -> pin to read IR value from
  OUT -> true = good(white surface)
         false = bad(black surface / no reflection)
  */
  bool val = (digitalRead(sensorPin) == HIGH);
  Serial.print("> IR sensor on pin # ");
  Serial.print(sensorPin);
  Serial.println("  working <");
  return val;
}

int turnXdegrees(bool turn_right = true, bool feedback90degree = true, int max_iterations = 100){
  /*
  turns vehicle by spinning both wheels in different direction untill US sensor
  detects target to left,
  IN  -> bool turn_right (determines turn-direction), set to false for left turns
         bool feedback (90 degree turn) if US is to be used, otherwise set max iterations
  OUT -> returns number of iterations to complete turn
  */
  int motorspeed = 200;
  int time_step  = 20;  // Size of time-stepping
  int pos;
  analogWrite(EN_A_pin, motorspeed);
  analogWrite(EN_B_pin, motorspeed);
  if (turn_right){
    pos = 0;
    //
    digitalWrite(motor_A_pin_1, LOW);
    digitalWrite(motor_A_pin_2, HIGH);
    digitalWrite(motor_B_pin_1, LOW);
    digitalWrite(motor_B_pin_2, HIGH);
  }
  else {
    pos = 180;
    //
    digitalWrite(motor_A_pin_1, HIGH);
    digitalWrite(motor_A_pin_2, LOW);
    digitalWrite(motor_B_pin_1, HIGH);
    digitalWrite(motor_B_pin_2, LOW);
  }
  int i = 0;
  if (feedback90degree){
    int object_distance = distanceUS(pos);
    while ( (object_distance > turning_radius + tol)  && (i < max_iterations) ){
      // uses US to see when turn is finished
      i++;
      delay(time_step);
      object_distance = distanceUS(pos);
    }
    stopMotors();
    if ((i == max_iterations) && (object_distance > turning_radius + tol)){
      Serial.println("ERROR! increase max_iterations or possible hardware bug");
      return 0;
    }
    else {
      return i;
    }
  }
  else {
    for (i = 0;i< max_iterations;i++){
      // turns blindly
       delay(time_step);
    }
    return i;
  }
}
