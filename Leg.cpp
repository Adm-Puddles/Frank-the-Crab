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
