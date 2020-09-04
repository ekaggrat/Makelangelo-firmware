//------------------------------------------------------------------------------
// Makelangelo - firmware for various robot kinematic models
// dan@marginallycelver.com 2013-12-26
// Please see http://www.github.com/MarginallyClever/makelangeloFirmware for more information.
//------------------------------------------------------------------------------

#include "configure.h"
#include "robot_scara.h"

#if MACHINE_STYLE == SCARA

#include <Arduino.h>


// use law of cosines property to find one interior angle in a triangle.  c=arccos((aa+bb-cc)/(2ab)).
float lawOfCosines(float a,float b,float c) {
  float numerator = sq(a)+sq(b)-sq(c);
  float denominator = 2.0*a*b;
  if(denominator==0) return 0;
  return acos(numerator/denominator);
}


/**
 * Inverse Kinematics turns XY coordinates into step counts from each motor
 * @param x cartesian coordinate
 * @param y cartesian coordinate
 * @param motorStepArray a measure of each belt to that plotter position
 */
void IK(const float *const cartesian, long *motorStepArray) {
  // see https://appliedgo.net/roboticarm/
  float x = cartesian[0];
  float y = cartesian[1];
  float z = cartesian[2];

  //BICEP_LENGTH_MM and FOREARM_LENGTH_MM are defined in robot_scara.h.
  // TODO save the numbers in EEPROM so they can be tweaked without a recompile?

  // from cartesian x,y we can get c, the distance from origin to x,y.
  // use law of cosines to
  float c = sqrt(sq(x) + sq(y));
  // then law of cosines will give us the elbow angle
  float elbowAngle = lawOfCosines(BICEP_LENGTH_MM, FOREARM_LENGTH_MM, c);

  // shoulder angle is made of two parts: the interior corner inside the
  float shoulderAngle = atan2(y, x) + lawOfCosines(c, BICEP_LENGTH_MM, FOREARM_LENGTH_MM);

  // angles are in radians.  we need degrees
  //motorStepArray[0] = lround(shoulderAngle * TODEGREES / MICROSTEP_PER_DEGREE);
  //motorStepArray[1] = lround(elbowAngle * TODEGREES / MICROSTEP_PER_DEGREE);

  motorStepArray[NUM_MOTORS] = z;


  // my code

  float L1 = 80; // the length of the shoulder
  float L2 = 100; // the length of the elbow
  float L0 = 35; // gap between arms
  float A1 = acos((sq(L1) + (sq(L0 + x) + sq(y)) - sq(L2)) / ((2 * L1) * sqrt(sq(L0 + x)  + sq(y))));
  float A2 = acos((sq(L1) + (sq(L0 - x) + sq(y)) - sq(L2)) / ((2 * L1) * sqrt(sq(L0 - x)  + sq(y))));
  float BB1 = atan(y / (L0 + x));
  float BB2 = atan(y / (L0 - x));

  float theta1 = BB1 + A1;
  float theta2 = 3.141516 - BB2 - A2;


  double elbow_ang = theta1;
  double shoulder_ang = theta2;

  motorStepArray[0] = lround(shoulder_ang / MICROSTEP_PER_DEGREE) ;
  motorStepArray[1] = lround (elbow_ang / MICROSTEP_PER_DEGREE) ;
}



/**
   Forward Kinematics - turns step counts into XY coordinates
   @param motorStepArray a measure of each belt to that plotter position
   @param axies the resulting cartesian coordinate
   @return 0 if no problem, 1 on failure.
*/
int FK(long *motorStepArray, float *axies) {
  float a = motorStepArray[0] * MICROSTEP_PER_DEGREE * TORADIANS;
  float b = motorStepArray[1] * MICROSTEP_PER_DEGREE * TORADIANS;
    float L1 = 80; // the length of the shoulder
  float L2 = 100; // the length of the elbow
  float L0 = 35; // gap between arms

  
   float x1 = cos(a)*L1;
   float y1 = sin(a)*L1;
   float x2 = cos(b)*L1+(L0*2);
   float y2 = sin(b)*L1;
   float DD = sqrt(sq(x2-x1)+sq(y2-y1)); 
   float Lm = atan2((y2-y1),(x2-x1));
   float Mu = acos(DD/(2*L2));
   axies[0] = x1+(L2*cos(Lm+Mu));
   axies[1] = y1+(L2*sin(Lm+Mu));
   

  //axies[0] = cos(a) * BICEP_LENGTH_MM - cos(b) * FOREARM_LENGTH_MM;
  //axies[1] = sin(a) * BICEP_LENGTH_MM - sin(b) * FOREARM_LENGTH_MM;
  axies[2] = motorStepArray[NUM_MOTORS];
}



void robot_findHome() {
  wait_for_empty_segment_buffer();
  motor_engage();

  findStepDelay();

  Serial.println(F("Finding..."));

  uint8_t i, hits;
  // back up until all switches are hit
  do {
    hits = 0;
    // for each stepper,
    for (ALL_MOTORS(i)) {
      digitalWrite(motors[i].dir_pin, HIGH);
      // if this switch hasn't been hit yet
      if (digitalRead(motors[i].limit_switch_pin) == HIGH) {
        // move "down"
        Serial.print('|');
        digitalWrite(motors[i].step_pin, HIGH);
        digitalWrite(motors[i].step_pin, LOW);
      } else {
        ++hits;
        Serial.print('*');
      }
    }
    Serial.println();
    pause(step_delay);
  } while (hits < NUM_MOTORS);
  Serial.println(F("Found."));
  
  float zeros[2] = {0, 0};
  teleport(zeros);
}


void robot_setup() {}

#endif  // SCARA
