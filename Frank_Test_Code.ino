/*
Test code for F.R.A.N.K the crab (Fully. Robotic. Autonomous. Nonsense. Krab the crab)
Can feed target xyz coordinates into the Leg class through an array, currently has no way of accepting input 
Still need to move the moveServo function into the leg class without it breaking the code
Still need to figure out how to get a waypoint system working before I get a concussion from banging my head on the desk
*/


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
double Coxa_Length = 38.25;
double Femur_Length = 80;
double Tibia_Length = 167;

// Offset origin in mm to resting stance
const int X_Offset = 0;
const int Y_Offset = 125;
const int Z_Offset = -130;

// Range of motion in degrees of servos
int Servo_Minimum = 0;
int Servo_Maximum = 180;

// Min and Max PWM signal for servos
int PWM_Minimum = 500 * 4;
int PWM_Maximum = 2500 * 4;

// Create an instance of the leg class for each leg
Leg l1(0, 1, 2, Coxa_Length, Femur_Length, Tibia_Length, PWM_Minimum, PWM_Maximum, Servo_Minimum, Servo_Maximum);
Leg l2(3, 4, 5, Coxa_Length, Femur_Length, Tibia_Length, PWM_Minimum, PWM_Maximum, Servo_Minimum, Servo_Maximum);
Leg l3(6, 7, 8, Coxa_Length, Femur_Length, Tibia_Length, PWM_Minimum, PWM_Maximum, Servo_Minimum, Servo_Maximum);
Leg l4(9, 10, 11, Coxa_Length, Femur_Length, Tibia_Length, PWM_Minimum, PWM_Maximum, Servo_Minimum, Servo_Maximum);
Leg l5(12, 13, 14, Coxa_Length, Femur_Length, Tibia_Length, PWM_Minimum, PWM_Maximum, Servo_Minimum, Servo_Maximum);
Leg l6(15, 16, 17, Coxa_Length, Femur_Length, Tibia_Length, PWM_Minimum, PWM_Maximum, Servo_Minimum, Servo_Maximum);

// Arrays for X, Y, Z target coordinates for 6 legs.
double l1_Target[3];
double l2_Target[3];
double l3_Target[3];
double l4_Target[3];
double l5_Target[3];
double l6_Target[3];

void setup() {

  maestroSerial.begin(9600);
  Serial.begin(9600);
}

void loop() {

  l1.calculateIK(l1_Target[0], l1_Target[1], l1_Target[2]);

  // Call move servos function for each leg 
  moveServos(l1.getCoxaPin(), l1.getTheta1Target());
  moveServos(l1.getFemurPin(), l1.getTheta2Target());
  moveServos(l1.getTibiaPin(), l1.getTheta3Target());
}

void moveServos(int pin, double target) {
  maestro.setTarget(pin, target);
}

void createWaypoints () {

}
