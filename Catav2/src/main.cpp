/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// L1                   motor         2               
// L2                   motor         3               
// L3                   motor         4               
// R1                   motor         5               
// R2                   motor         6               
// R3                   motor         7               
// Head                 inertial      18              
// LTrack               rotation      20              
// RTrack               rotation      19              
// Single               digital_out   A               
// Double               digital_out   B               
// Cata                 motor         8               
// CataArm              rotation      9               
// Intake               motor         11              
// roller               optical       12              
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "Odometry.h"
#include "Controls.h"
#include "Functions.h"

using namespace vex;

// A global instance of competition
competition Competition;

double DesiredAng(int angle) {
 double CheckL;
 
 if(angle > OdomHeading) {
   CheckL = std::abs(angle - OdomHeading);
  } else{
   CheckL = (360-OdomHeading)+angle;
  }
 
 double CheckR;

  if(angle > OdomHeading) {
   CheckR = (360-angle)+OdomHeading;
  } else{
   CheckR = (OdomHeading - angle);
  }

  double Heading;
  if(CheckL > std::abs(CheckR)) {
   Heading = CheckR;
    } else {
   Heading = -CheckL;
  }
  
  
  //std::cout << "     Left Turn is " << CheckL;
  //std::cout << "     Right Turn is " << CheckR;
  return Heading;
}

double kP = 8;
double kD = 0;

double turnkP = .25;
double turnkD = 0;
double turnkI = 0;
double desy;
double error; 
double prevError = 0; //Position 20 miliseconds ago
double derivative; // error - prevError : Speed

double turnerror; 
double turnprevError; //Position 20 miliseconds ago
double turnderivative; // error - prevError : Speed
double totalturnError = 0;
double reversal;


int YPID(double desy){
  while(true){
    error = YPos - desy;

    double VertSpeed = (error * kP) + (derivative * kD);
    VertMov(VertSpeed);
  
    if(InRange(VertSpeed, -0.5, 0.5)){
      break;
    }

  }
  L1.stop(hold);
  L2.stop(hold);
  L3.stop(hold);
  R1.stop(hold);
  R2.stop(hold);
  R3.stop(hold);
  return 1;
}

int XPID(int desy){
  while(true){
    error = XPos - desy;

    double VertSpeed = (error * kP) + (derivative * kD);
    VertMov(VertSpeed);
  
    if(InRange(VertSpeed, -0.5, 0.5)){
      break;
    }

  }
  L1.stop(hold);
  L2.stop(hold);
  L3.stop(hold);
  R1.stop(hold);
  R2.stop(hold);
  R3.stop(hold);
  return 1;
}

int turnPID(int desy){
  while(true){
    //turnerror = OdomHeading - desy;
    double new_error =(DesiredAng(desy)); 
    /*if (new_error < 1){
      reversal = -1;
    } else {
      reversal = 1;
    }
    */
    double abs_error = std::abs(new_error);
    std::cout << new_error << std::endl;
    double TurnSpeed = 0;
    if(InRange(abs_error, 100, 360) == 1){
    TurnSpeed = 10;
    }
    if(InRange(abs_error, 50, 100) == 1){
    TurnSpeed = 6;
    }
    if(InRange(abs_error, 10, 50) == 1){
    TurnSpeed = 3;
    }
    if(InRange(abs_error, 1, 10) == 1){
    TurnSpeed = 2.45;
    }
    //std::cout << TurnSpeed << std::endl;
    //std::cout << reversal << std::endl;
    if(InRange(new_error, 0, 1) == 1){
    TurnSpeed = 0;
    L1.stop(hold);
    L2.stop(hold);
    L3.stop(hold);
    R1.stop(hold);
    R2.stop(hold);
    R3.stop(hold);
    }
    TurnMov(TurnSpeed);
    /*
    if(TurnSpeed == 0){
      break;
    }
   */
  }
  L1.stop(hold);
  L2.stop(hold);
  L3.stop(hold);
  R1.stop(hold);
  R2.stop(hold);
  R3.stop(hold);
  return 1;
}

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  LTrack.setPosition(0, degrees);
  RTrack.setPosition(0, degrees);
  CataArm.setPosition(0, degrees);
  X = 0;
  Y = 0;

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
 vex::task Odometry (TrackPOS);
 wait(3, sec);
 turnPID(180);
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  // User control code here, inside the loop
  while (1) {
    Drive();
    Catapult();
    Intaker();
    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
