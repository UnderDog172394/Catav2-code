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

double VertMov(double speed){
 L1.spin(forward, speed, voltageUnits::volt);
 L2.spin(forward, speed, voltageUnits::volt);
 L3.spin(forward, speed, voltageUnits::volt);
 R1.spin(forward, speed, voltageUnits::volt);
 R2.spin(forward, speed, voltageUnits::volt);
 R3.spin(forward, speed, voltageUnits::volt);
  return 1;
}

double LeftMov(double speed){
 L1.spin(forward, speed, voltageUnits::volt);
 L2.spin(forward, speed, voltageUnits::volt);
 L3.spin(forward, speed, voltageUnits::volt);
 return 1;
}

double RightMov(double speed){
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

int RollerStop(){
 L1.stop();
 L2.stop();
 L3.stop();
 R1.stop();;
 R2.stop();
 R3.stop();
  return 1;
}

int autoroller(){
 while(true){
   if(roller.color() == red && roller.isNearObject()) {
    RollerStop();
    Intake.stop();
    break;
   } else {
   RollerVertMov();
   Intake.spin(forward, -6, voltageUnits::volt);
    }  
  }
return 1;
}



int CataSet(){
  while((CataArm.position(degrees) < 64) == 1){
    Cata.spin(forward, 12, voltageUnits::volt);
    std::cout << CataArm.position(degrees) << std::endl;
  }
  Cata.stop();
  return 1;
}

int CataFire(){
  Cata.spin(forward, 12, voltageUnits::volt);
  wait(.5, sec);
  Cata.stop();
  return 1;
}

int StartInt(){
  Intake.spin(forward, -12, voltageUnits::volt);
  return 1;
}

int StopInt(){
  Intake.stop();
  return 1;
}

int sgn2(double input) {
  if (input > 0)
    return 1;
  else if (input < 0)
    return -1;
  return 0;
}

int boostON(){



  return 1;
}

int boostOFF(){



  return 1;
}

double VertMov2(double speed){
 L1.spin(forward, speed, voltageUnits::volt);
 L2.spin(forward, speed, voltageUnits::volt);
 L3.spin(forward, speed, voltageUnits::volt);
 R1.spin(forward, speed, voltageUnits::volt);
 R2.spin(forward, speed, voltageUnits::volt);
 R3.spin(forward, speed, voltageUnits::volt);
 wait(.5, sec);
 L1.stop();
 L2.stop();
 L3.stop();
 R1.stop();
 R2.stop();
 R3.stop();
  return 1;
}