#include <Arduino.h>
#include "Leg.h"
#include <PololuMaestro.h>

Leg::Leg(int coxa_pin, int femur_pin, int tibia_pin, double coxa_length, double femur_length, double tibia_length, double minimum_target, double maximum_target, double minimum_angle, double maximum_angle) {
    _coxa_pin = coxa_pin;
    _femur_pin = femur_pin;
    _tibia_pin = tibia_pin;

    _coxa_length = coxa_length;
    _femur_length = femur_length;
    _tibia_length = tibia_length;  

    _minimum_target = minimum_target;
    _maximum_target = maximum_target;
    _minimum_angle = minimum_angle;
    _maximum_angle = maximum_angle;  
}


///////////////////////////////////////////////////
// Calculate target angles for the leg class using X,Y,Z targets and update theta variables in the leg class
///////////////////////////////////////////////////

void Leg::calculateIK(double x, double y, double z){
    theta1 = atan(x / y);  //Find Theta 1
    double h = sqrt( pow(y, 2) + pow(x, 2) );
    double L = sqrt( pow(h, 2) + pow(z, 2) );  //Find hypotenus
    theta3 = acos(  ( pow(getFemurLength(), 2) + pow(getTibiaLength(), 2) - pow(L, 2) ) / (2 * getFemurLength() * getTibiaLength() ));  // Find theta 3
    double b = acos(  ( pow(L, 2) + pow(getFemurLength(), 2) - pow(getTibiaLength(), 2) ) / (z * L * getFemurLength() ));
    double a = atan(z / h);
    theta2 = b + a;
}


////////////////////////////////////////////////
// Convert degrees to PWM signal for servo positioning
////////////////////////////////////////////////

double Leg::degreesToTarget(double degree, double minimum_target, double maximum_target, double minimum_angle, double maximum_angle) {
  return map(degree, getMinimumAngle(), getMaximumAngle(), getMinimumTarget(), getMaximumTarget());
}


//////////////////////////////////////////////////////////
// Move servos to target positions
//////////////////////////////////////////////////////////

void Leg::convertDegreesToTarget() {
  // Convert target angle to PWM signal
  float theta1_target = degreesToTarget(radiansToDegrees(theta1) + 90, _minimum_target, _maximum_target, _minimum_angle, _maximum_angle);
  float theta2_target = degreesToTarget(radiansToDegrees(theta2) + 90, _minimum_target, _maximum_target, _minimum_angle, _maximum_angle);
  float theta3_target = degreesToTarget(radiansToDegrees(theta3) + 90, _minimum_target, _maximum_target, _minimum_angle, _maximum_angle);
}


//////////////////////////////////////////////////////////
// Its in the name. If you need a comment here, your beyond help
//////////////////////////////////////////////////////////

double Leg::radiansToDegrees(double radians) {
  return double(radians * 180.0 / PI);
}


//////////////////////////////////////////////////////////
// See above, seriously....
// Not actually used but seemed too useful to get rid of
//////////////////////////////////////////////////////////

double Leg::degreesToRadians(double degrees) {
  double radians = degrees * (PI / 180);
  return radians;
}
