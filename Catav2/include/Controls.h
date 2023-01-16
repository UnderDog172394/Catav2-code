//Controller Controls
#include "vex.h"
#include <math.h>
#include <iostream> 
#include <cmath>

using namespace vex;

//exponential function
double AxisSens(double axis){
    
    double newaxis;
    
    if(axis < 0){
        newaxis = (1.2*pow(1.043, std::abs(axis)) - 1.2 + 0.2*std::abs(axis))*-1;
    } else {
        newaxis = (1.2*pow(1.043, axis) - 1.2 + 0.2*axis);
    }
    return newaxis; 
}

//deadzone
double AxisDead(double axis, int deadzone){
    
    double final_axis;
    
    //checks if axis is positive of negative
    if(axis > 0){
        if(axis > deadzone){
        final_axis = axis;
    } else {
        final_axis = 0;
    } 
    
    } else {
        
        if(axis < -deadzone){
        final_axis = axis;
    } else {
        final_axis = 0;
    } 
    } 
    return final_axis;
}


void Drive(){
  int axis3 = Controller1.Axis3.position(percent);
  int axis2 = Controller1.Axis2.position(percent);
  int deadzone = 0;

  double axis03 = AxisDead(axis3, deadzone);
  double Finalaxis3 = AxisSens(axis03);

  double axis02 = AxisDead(axis2, deadzone);
  double Finalaxis2 = AxisSens(axis02);

  double LeftVal = Finalaxis2;
  double LeftVolts = LeftVal * -0.12;
    
  double RightVal = Finalaxis3;
  double RightVolts = RightVal * -0.12;

  L1.spin(forward, LeftVolts, voltageUnits::volt);
  L2.spin(forward, LeftVolts, voltageUnits::volt);
  L3.spin(forward, LeftVolts, voltageUnits::volt);

  R1.spin(forward, RightVolts, voltageUnits::volt);
  R2.spin(forward, RightVolts, voltageUnits::volt);
  R3.spin(forward, RightVolts, voltageUnits::volt);
}
/*
void pnuematic(){
if(Controller1.ButtonA.pressing()){
 Single.set(1);
} else{
  Single.set(0);
}
if(Controller1.ButtonB.pressing()){
 Double.set(1);
} else{
  Double.set(0);
}
}
*/

bool CataPos (int x) {
  return (x >= 63 && x < 65);
}



void Catapult(){
  if(Controller1.ButtonA.pressing()){
      Cata.spin(forward, 12, voltageUnits::volt);

    } else if(CataArm.angle() >= 58 ){
      Cata.stop(hold);
     } else {
      Cata.spin(forward, 12, voltageUnits::volt);
    }
  
}

void Intaker(){
if(Controller1.ButtonR1.pressing()){
      Intake.spin(forward, 12, voltageUnits::volt);
    } else if( Controller1.ButtonL1.pressing()) {
     Intake.spin(forward, -12, voltageUnits::volt);
    } else {
      Intake.stop();
    }

    
       
}