#include <PololuMaestro.h>
#include <Leg.h>
#ifdef SERIAL_PORT_HARDWARE_OPEN
#define maestroSerial SERIAL_PORT_HARDWARE_OPEN
#else
#include <SoftwareSerial.h>
SoftwareSerial maestroSerial(0, 1);
#endif

MiniMaestro maestro(maestroSerial);


// Leg segment lengths
double Coxa_Len = 38.25;
double Femur_Len = 80;
double Tibia_Len = 167;

// Offset origin in mm to resting stance
const int X_Offset = 0;
const int Y_Offset = 125;
const int Z_Offset = -130;

// Range of motion in degrees of servos
int Servo_Min = 0;
int Servo_Max = 180;

// Min and Max PWM signal for servos
int PWM_Min = 500 * 4;
int PWM_Max = 2500 * 4;

// Create an instance of the leg class for each leg
Leg l1(0, 1, 2, Coxa_Len, Femur_Len, Tibia_Len);
Leg l2(3, 4, 5, Coxa_Len, Femur_Len, Tibia_Len);
Leg l3(6, 7, 8, Coxa_Len, Femur_Len, Tibia_Len);
Leg l4(9, 10, 11, Coxa_Len, Femur_Len, Tibia_Len);
Leg l5(12, 13, 14, Coxa_Len, Femur_Len, Tibia_Len);
Leg l6(15, 16, 17, Coxa_Len, Femur_Len, Tibia_Len);

// Arrays for X, Y, Z target coordinates for 6 legs.
double l1Target[3];
double l2Target[3];
double l3Target[3];
double l4Target[3];
double l5Target[3];
double l6Target[3];

void setup() {

  maestroSerial.begin(9600);
  Serial.begin(9600);
}

void loop() {
  // l1.targetX = 0;
  // Serial.print(l1.targetX);
  // Serial.print("\n");
  // l1.targetX = 10;
  // Serial.print(l1.targetX);
  // Serial.print("\n");

  // Call calculate IK function to set joint angles for each leg based on XYZ target coordinates
  calculateIK(l1, l1Target[0], l1Target[1], l1Target[2]);
  calculateIK(l2, l2Target[0], l2Target[1], l2Target[2]);
  calculateIK(l3, l3Target[0], l3Target[1], l3Target[2]);
  calculateIK(l4, l4Target[0], l4Target[1], l4Target[2]);
  calculateIK(l5, l5Target[0], l5Target[1], l5Target[2]);
  calculateIK(l6, l6Target[0], l6Target[1], l6Target[2]);

  // Call move servos function for each leg 
  moveServos(l1);
  moveServos(l2);
  moveServos(l3);
  moveServos(l4);
  moveServos(l5);
  moveServos(l6);
}


///////////////////////////////////////////////////
// Calculate target angles for the leg class using X,Y,Z targets and update theta variables in the leg class
///////////////////////////////////////////////////
void calculateIK(Leg &leg, double x, double y, double z) {
  leg.theta1 = atan(x / y);  //Find Theta 1
  double h = sqrt( pow(y, 2) + pow(x, 2) );
  double L = sqrt( pow(h, 2) + pow(z, 2) );                                                                                                //Find hypotenus
  leg.theta3 = acos(  ( pow(leg.getFemurLen(), 2) + pow(leg.getTibiaLen(), 2) - pow(L, 2) ) / (2 * leg.getFemurLen() * leg.getTibiaLen() ));  // Find theta 3
  double b = acos(  ( pow(L, 2) + pow(leg.getFemurLen(), 2) - pow(leg.getTibiaLen(), 2) ) / (z * L * leg.getFemurLen() ));
  double a = atan(z / h);
  leg.theta2 = b + a;
}

////////////////////////////////////////////////
// Convert degrees to PWM signal for servos positioning
////////////////////////////////////////////////
double degreesToTarget(double degree, double minTarget, double maxTarget, double minAngle, double maxAngle) {
  return map(degree, minAngle, maxAngle, minTarget, maxTarget);
}

//////////////////////////////////////////////////////////
// Its in the name. If you need a comment here, your beyond help
//////////////////////////////////////////////////////////
double radiansToDegrees(double rads) {
  return double(rads * 180.0 / PI);
}

//////////////////////////////////////////////////////////
// See above, seriously....
//////////////////////////////////////////////////////////
double degreesToRadians(double degrees) {
  double rads = degrees * (PI / 180);
  return rads;
}

//////////////////////////////////////////////////////////
// Move servos to target positions
//////////////////////////////////////////////////////////
void moveServos(Leg &leg) {
  float theta1 = degreesToTarget(radiansToDegrees(leg.theta1) + 90, PWM_Min, PWM_Max, Servo_Min, Servo_Max);
  float theta2 = degreesToTarget(radiansToDegrees(leg.theta2) + 90, PWM_Min, PWM_Max, Servo_Min, Servo_Max);
  float theta3 = degreesToTarget(radiansToDegrees(leg.theta3) + 90, PWM_Min, PWM_Max, Servo_Min, Servo_Max);

    // Move servos to target position
  maestro.setTarget(leg.getCoxaPin(), theta1);
  maestro.setTarget(leg.getFemurPin(), theta2);
  maestro.setTarget(leg.getTibiaPin(), theta3);
}
