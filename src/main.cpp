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
// roller               optical       10              
// Right                digital_out   E               
// Left                 digital_out   F               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "Odometry.h"
#include "Controls.h"
#include "Functions.h"

using namespace vex;

// A global instance of competition
competition Competition;

double conv = Pi/180;

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
//turnPID
double turnkp = 0;
double turnki = 0.06;
double turnkd = 0.31;

double turnerror = 0;
double turnderivative = 0;
double turnintegral = 0;

double preverror = 0;

double turnPID(int desy){
  double turnerror2 = -(DesiredAng(desy));
  turnkp = ((.00001111111*pow(std::abs(turnerror2), 2)) - (.00321111*std::abs(turnerror2)) + 0.362);
  //std::cout<< turnkp ;
  while(true){
    turnerror = -(DesiredAng(desy));
    
    //std::cout<< turnerror << "  ";
    
    turnderivative = turnerror - preverror;
    
    if(std::abs(turnintegral) > 12){
      turnintegral = 0;
    } else {
      turnintegral += turnerror;
      if (turnki != 0) {
      if (std::fabs(turnerror) < turnki){
        turnintegral += turnerror;
      }
      if(sgn2(turnerror) != sgn2(preverror)){
      turnintegral = 0;
      }
    }  
    }
    
    
    double TurnSpeed = (turnerror * turnkp) + (turnintegral * turnki) + (turnderivative * turnkd);
    /*
    std::cout<< turnerror * turnkp << "   " ;
    std::cout<< turnintegral * turnki << "   " ;
    std::cout<< turnderivative * turnkd << std::endl;
    */
    
    //std::cout<< TurnSpeed << std::endl;
    
    TurnMov(TurnSpeed);
    
    if(InRange(TurnSpeed, -0.5, 0.5)){
      break;
    }
   preverror = turnerror;
   wait(10, msec);
  }
  L1.stop(hold);
  L2.stop(hold);
  L3.stop(hold);
  R1.stop(hold);
  R2.stop(hold);
  R3.stop(hold);
  
  return 1;
}
double Correction = 0;
double CorrectionSpeed = 0;

int CorrectionPID(){
  double turnerror2 = -(DesiredAng(Correction));
  turnkp = ((.00001111111*pow(turnerror2, 2)) - (.00321111*turnerror2) + 0.362);
  while(true){
    turnerror = -(DesiredAng(Correction));

    turnderivative = turnerror - preverror;

    if(turnintegral > 12){
      turnintegral = 0;
    } else {
      turnintegral += turnerror;
      if (turnki != 0) {
      if (std::fabs(turnerror) < turnki){
        turnintegral += turnerror;
      }
      if(sgn2(turnerror) != sgn2(preverror)){
      turnintegral = 0;
      }
    }  
    }
    

    CorrectionSpeed = (turnerror * turnkp) + (turnintegral * turnki) + (turnderivative * preverror);
    
   preverror = turnerror;
   wait(10, msec);
  }
  return 1;
}

double kP = 8.2;
double kD = 0;

double desy;
double error; 
double prevError = 0; //Position 20 miliseconds ago
double derivative; // error - prevError : Speed

double reversal;


int YPID(double desy){
  Correction = OdomHeading;
  while(true){
    vex::task Correctional (CorrectionPID);
    error = std::abs(YPos) - std::abs(desy);
    //std::cout << error << "  " ;
    derivative = error - prevError;

    double VertSpeed = (error * kP) + (derivative * kD);
    //std::cout << VertSpeed << std::endl;
    

    LeftMov(VertSpeed + CorrectionSpeed);

    RightMov(VertSpeed - CorrectionSpeed);


    
  
    if(InRange(VertSpeed, -0.9, 0.9)){
     break;
    }

    prevError = error;
    wait(10, msec);  
  }
  vex::task::stop (CorrectionPID);
  L1.stop(hold);
  L2.stop(hold);
  L3.stop(hold);
  R1.stop(hold);
  R2.stop(hold);
  R3.stop(hold);
  return 1;
}

int XPID(double desy){
  Correction = OdomHeading;
  std::cout<< XPos << std::endl;
  while(true){
    vex::task Correctional (CorrectionPID);
    error = std::abs(XPos) - std::abs(desy);
    std::cout << error << std::endl ;
    derivative = error - prevError;

    double VertSpeed = (error * kP) + (derivative * kD);
    //std::cout << VertSpeed << std::endl;
    LeftMov(VertSpeed + CorrectionSpeed);

    RightMov(VertSpeed - CorrectionSpeed);
  
    if(InRange(VertSpeed, -0.9, 0.9)){
     break;
    }

    prevError = error;
    wait(10, msec); 
  }
  vex::task::stop (CorrectionPID);
  L1.stop(hold);
  L2.stop(hold);
  L3.stop(hold);
  R1.stop(hold);
  R2.stop(hold);
  R3.stop(hold);
  return 1;
}

int ForwardPID(double desl){
    double HRad = OdomHeading*conv;
    double desy = (desl*cos(HRad));
    double desx = (desl*sin(HRad));
    std::cout << desy << std::endl;
    std::cout << desx << std::endl;
    if (std::abs(desy) >= std::abs(desx)){
      std::cout << "running y" << std::endl;
      YPID(desy);
    } 
    if (std::abs(desx) > std::abs(desy)) {
      std::cout << "running x" << std::endl;
      XPID(desx);
    }
 return 1;   
}



int RESET(double seconds){
  X = 0;
  Y = 0;
  wait(seconds, sec);
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
 autoroller();
 VertMov2(-3);
 RESET(1);
 turnPID(45);
 RESET(1);
 ForwardPID(5);
 turnPID(48);
 RESET(1);
 ForwardPID(6);
 turnPID(270);
 autoroller();
 
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
    Endgame();
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
