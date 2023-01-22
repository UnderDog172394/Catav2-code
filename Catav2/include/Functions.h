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
double TurnMov(double speed){
 L1.spin(forward, speed, voltageUnits::volt);
 L2.spin(forward, speed, voltageUnits::volt);
 L3.spin(forward, speed, voltageUnits::volt);
 R1.spin(forward, -speed, voltageUnits::volt);
 R2.spin(forward, -speed, voltageUnits::volt);
 R3.spin(forward, -speed, voltageUnits::volt);
  return 1;
}

int RollerVertMov(){
 L1.spin(forward, 4, voltageUnits::volt);
 L2.spin(forward, 4, voltageUnits::volt);
 L3.spin(forward, 4, voltageUnits::volt);
 R1.spin(forward, 4, voltageUnits::volt);
 R2.spin(forward, 4, voltageUnits::volt);
 R3.spin(forward, 4, voltageUnits::volt);
  return 1;
}



int autoroller(){
roller.setLightPower(50, pct);
task myTask = task(RollerVertMov);
Intake.spin(forward, 12, voltageUnits::volt);
if(roller.color() == red){
wait(1, sec);
Intake.stop();
}



return 1;
}
