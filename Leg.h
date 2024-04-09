#include <Arduino.h>
#include <PololuMaestro.h>
#include <vector>
#ifndef Leg_h
#define Leg_h

class Leg {
  public:
    Leg(int coxa_pin, int femur_pin, int tibia_pin, double coxa_length, double femur_length, double tibia_length, double minimum_target, double maximum_target, double minimum_angle, double maximum_angle, double angle_offset, int num_waypoints, double max_speed);

    double theta1;
    double theta2;
    double theta3;
    double theta1_target;
    double theta2_target;
    double theta3_target;

    double degreesToTarget(double degree, double minTarget, double maxTarget, double minAngle, double maxAngle);
    double radiansToDegrees(double radians);
    double degreesToRadians(double degrees);

    void calculateIK(double x, double y, double z);

    void convertDegreesToTarget();

    void createWaypoint(double r, double theta, double phi);
    void createWaypointArray(int num_waypoints, double r, double theta);

    void moveToWaypoint(double max_speed);
    double getXYZDistance (double x1, double y1, double z1, double x2, double y2, double z2);
    double getMaxDistance();
    double getTargetDistance(int pin);
    void setSpeed();
    void atTarget();

    int getCoxaPin() {return _coxa_pin;}
    int getFemurPin() {return _femur_pin;}
    int getTibiaPin() {return _tibia_pin;}

    double getCoxaLength() {return _coxa_length;}
    double getFemurLength() {return _femur_length;}
    double getTibiaLength() {return _tibia_length;}

    double getMinimumTarget() {return _minimum_target;}
    double getMaximumTarget() {return _maximum_target;}
    double getMinimumAngle() {return _minimum_angle;}
    double getMaximumAngle() {return _maximum_angle;}

    double getTheta1Target() {return theta1_target;}
    double getTheta2Target() {return theta2_target;}
    double getTheta3Target() {return theta3_target;}

    double getAngleOffset() {return _angle_offset;}

    double getMaxSpeed() {return _max_speed;}

  private:
    int num_waypoints; 
    int _coxa_pin, _femur_pin, _tibia_pin;
    double _coxa_length, _femur_length, _tibia_length, _minimum_target, _maximum_target, _minimum_angle, _maximum_angle, _angle_offset, _max_speed;
    double waypoint[3];
    vector<array<double, 3>> waypoints_vector;

};

#endif
