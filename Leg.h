#include <Arduino.h>
#include <PololuMaestro.h>
#ifndef Leg_h
#define Leg_h

class Leg {
  public:
    Leg(int coxa_pin, int femur_pin, int tibia_pin, double coxa_len, double femur_len, double tibia_len);
    // void calculateIK(double x, double y, double z);
    double theta1;
    double theta2;
    double theta3;
    int getCoxaPin() {return _coxa_pin;}
    int getFemurPin() {return _femur_pin;}
    int getTibiaPin() {return _tibia_pin;}
    double getCoxaLen() {return _coxa_len;}
    double getFemurLen() {return _femur_len;}
    double getTibiaLen() {return _tibia_len;}

  private:
    int _coxa_pin, _femur_pin, _tibia_pin;
    double _coxa_len, _femur_len, _tibia_len;
};
#endif