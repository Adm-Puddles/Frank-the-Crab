#include <Arduino.h>
#include "Leg.h"
#include <PololuMaestro.h>
#include <vector>

Leg::Leg(int coxa_pin, int femur_pin, int tibia_pin, double coxa_length, double femur_length, double tibia_length, double minimum_target, double maximum_target, double minimum_angle, double maximum_angle, double angle_offset, int num_waypoints, double max_speed) {
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

    _angle_offset = angle_offset;

    _max_speed = max_speed;
}


///////////////////////////////////////////////////
// Calculate target angles for the leg class using XYZ targets and update theta variables in the leg class
///////////////////////////////////////////////////

void Leg::calculateIK(double x, double y, double z){
    theta1 = atan(x / y);  //Find Theta 1
    double h = (sqrt( pow(y, 2) + pow(x, 2) - getCoxaLength() ) );
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
// Convert degrees to 1/4 PWM signal from theta 1 2 and 3
//////////////////////////////////////////////////////////

void Leg::convertDegreesToTarget() {
  // Convert target angle to PWM signal
  float theta1_target = degreesToTarget(radiansToDegrees(theta1) + 90, _minimum_target, _maximum_target, _minimum_angle, _maximum_angle);
  float theta2_target = degreesToTarget(radiansToDegrees(theta2) + 90, _minimum_target, _maximum_target, _minimum_angle, _maximum_angle);
  float theta3_target = degreesToTarget(radiansToDegrees(theta3) + 90, _minimum_target, _maximum_target, _minimum_angle, _maximum_angle);
}


//////////////////////////////////////////////////////////
// Find XYZ coordinate and save to holding array
//////////////////////////////////////////////////////////

void Leg::createWaypoint(double r, double theta, double phi){
  waypoint[0] = r * sin(theta) * cos(phi); // X coordinate
  waypoint[1] = r * sin(theta) * sin(phi); // Y coordinate
  waypoint[2] = r * cos(theta);  // Z coordinate
 }


 //////////////////////////////////////////////////////////
 // Create waypoint array and cycle through
 //////////////////////////////////////////////////////////
void Leg::createWaypointArray (int num_waypoints, double r, double theta, double phi, bool onGround) {

  while (onGround) {
    double waypoint_length = 2r / num_waypoints; 
    for(int i = 0, i <= num_waypoints, i++) {
      waypoints_vector.push_back(createWaypoint(r, theta, phi));
      r -= waypoint_length;
    }

  } 
}


//////////////////////////////////////////////////////////
// Move servos to target
//////////////////////////////////////////////////////////
bool Leg::moveToWaypoint () {
  bool leg_at_target;
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
