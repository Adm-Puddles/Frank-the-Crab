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
 // Note: This is probably bulky. Look at splitting vector creation and moving through the vector as their own seperate functions
 //       Currently does not account for edge cases such as first step or end step or account for variable servo speeds so each servo reaches target in sync
 //////////////////////////////////////////////////////////
void Leg::createWaypointArray (int num_waypoints, double radius, double theta) {

  bool onGround = true; 
// Create and move through waypoint vector while the leg is pushing along the ground
  while (onGround) {
    double phi = 0; // Asimuth of waypoint. A value of 0 means the tip of leg stays on the ground
    double r = radius; // Half the length of a step
    double waypoint_length = 2r / num_waypoints; // Find distance in mm between each waypoint given number of waypoints. more waypoints means more precise motions
    for(int i = 0; i <= num_waypoints; i++) {
      waypoints_vector.push_back(createWaypoint(radius, theta, phi));
      radius -= waypoint_length;
    }

    // Move servos through each vector element and erase element once waypoint has been reached. set onGround to false if at the end of the waypoints
    while (!waypoints_vector.empty()) {
      moveToWaypoint();
      if(atTarget()) {
        waypoints_vector.erase(waypoints_vector.begin());
      }
      if (waypoints_vector.empty()) {
        onGround = false;
      }
    }
  } 

  r = radius; // reset radius

// Create and move through waypoint vector while leg is lifting and moving to beginning of step
  while(!onGround) {
    double phi = 90; //Azimuth of waypoint
    double waypoint_angle = 180 / num_waypoints; // Calculate angle to be subtracted from 180 to create waypoints_vector
    for(int i = 0; i <= num_waypoints; i++) {
      waypoints_vector.push_back(createWaypoint(r, theta, phi));
      phi -= waypoint_angle;
    }

    while (!waypoints_vector.empty()) {
      moveToWaypoint();
      if(atTarget()) {
        waypoints_vector.erase(waypoints_vector.begin());
      }
      if (waypoints_vector.empty()) {
        onGround = true;
      }
    }
  }
}


///////////////////////////////////////////////////////////
// Check if servos are at target position
///////////////////////////////////////////////////////////

bool Leg::atTarget(){
  if (maestro.getMovingState(getCoxaPin()) == 0 && maestro.getMovingState(getFemurPin()) == 0 && maestro.getMovingState(getTibiaPin()) == 0){
  return true;
  }
}

//////////////////////////////////////////////////////////
// Move servos to target
//////////////////////////////////////////////////////////
void Leg::moveToWaypoint () {
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
