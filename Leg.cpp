#include "Arduino.h"
#include "Leg.h"
#include "pololuMaestro.h"

Leg::Leg(int coxa_pin, int femur_pin, int tibia_pin, double coxa_len, double femur_len, double tibia_len) {
    _coxa_pin = coxa_pin;
    _femur_pin = femur_pin;
    _tibia_pin = tibia_pin;

    _coxa_len = coxa_len;
    _femur_len = femur_len;
    _tibia_len = tibia_len;    
}
// void Leg::calculateIK(double x, double y, double z) {
//     double theta1 = atan(x / y); //Find Theta 1
//     double h = sqrt(pow(y, 2) + pow(x, 2));
//     double L = sqrt(pow(h, 2) + pow(z, 2)); //Find hypotenus
//     double theta3 = acos(   (pow(_femur_len, 2) + pow(_tibia_len, 2) - pow(L, 2))   /   (2 * _femur_len * _tibia_len)); // Find theta 3 
//     double b = acos((pow(L, 2) + pow(_femur_len, 2) - pow(_tibia_len, 2)) / (z * L * _femur_len));
//     double a = atan(z / h);
//     double theta2 = b + a;
// }