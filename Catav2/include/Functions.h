#include "vex.h"
#include <math.h>
#include <iostream> 
#include <cmath>

using namespace vex;

double clamp (double var, double lo, double hi) {     
    if (var < lo) {
     var = lo;   
    } else if (var > hi) {
     var = hi;   
    }     
 return var;   
}

bool InRange (double x, double lower, double upper) {
  return (x >= lower && x < upper);
}

int VertMov(double speed){
 L1.spin(forward, speed, voltageUnits::volt);
 L2.spin(forward, speed, voltageUnits::volt);
 L3.spin(forward, speed, voltageUnits::volt);
 R1.spin(forward, speed, voltageUnits::volt);
 R2.spin(forward, speed, voltageUnits::volt);
 R3.spin(forward, speed, voltageUnits::volt);
  return 1;
}
int TurnMov(double speed){
 L1.spin(forward, speed, voltageUnits::volt);
 L2.spin(forward, speed, voltageUnits::volt);
 L3.spin(forward, speed, voltageUnits::volt);
 R1.spin(forward, -speed, voltageUnits::volt);
 R2.spin(forward, -speed, voltageUnits::volt);
 R3.spin(forward, -speed, voltageUnits::volt);
  return 1;
}