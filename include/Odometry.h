#include "vex.h"
#include <math.h>
#include <iostream> 
#include <cmath>

using namespace vex;

#define Pi 3.14159265358979323846
#define fieldscale 1.66548042705
#define SL 3.09 //distance from tracking center to middle of left wheel
#define SR 3.09 //distance from tracking center to middle of right wheel
#define SS 7.75 //distance from tracking center to middle of the tracking wheel
#define WheelDiam 2.75 //diameter of all the wheels being used for tracking
#define tpr 360  //Degrees per single encoder rotation
double DeltaL,DeltaR,DeltaB,currentL,currentR,PreviousL,PreviousR,DeltaTheta,X,Y,Theta,DeltaXSide,DeltaYSide,SideChord,OdomHeading,XPos,YPos,DeltaHeading,PreviousHeading,DeltaThetawheel;
//double Heading = Head.rotation(degrees) + (0.0188535 * (Head.heading(degrees)) - 0.0469265);

//inertial sensor


int TrackPOS() {
  while(true){
   double Heading = Head.rotation() + (0.0188535 * (Head.rotation()) - 0.0469265);
      if (Heading > 360){
      Head.setRotation(0, degrees);
    } 
   currentR = LTrack.position(degrees);
   currentL = RTrack.position(degrees);
   //Creates variables for change in each side info in inches (12.9590697 is circumference of wheel)
   DeltaL = ((currentL - PreviousL) * 8.639379797) / tpr;
   DeltaR = ((currentR - PreviousR) * 8.639379797) / tpr;
   //DeltaB = ((currentB - PreviousB) * 12.9590697) / tpr;

   //Determines the change in angle of the robot using the rotational change in each side
   //DeltaThetawheel = (DeltaR - DeltaL) / (SL + SR);
   DeltaTheta = (Heading - PreviousHeading) * (Pi/180);

   //Creates an if/else statement to prevent NaN values from appearing and causing issues with calculation
   if(DeltaTheta == 0) {  //If there is no change in angle
    X += DeltaL * sin (Theta);
    Y += DeltaL * cos (Theta);
    //X += DeltaB * cos (Theta + 1.57079633);
    //Y += DeltaB * sin (Theta + 1.57079633);

   //If there is a change in angle, it will calculate the changes in X,Y from chords of an arc/circle.
   } else {  //If the angle changes
      SideChord = 2 * ((DeltaL / DeltaTheta) + SL) * sin (DeltaTheta / 2);
      //BackChord = 2 * ((DeltaB / DeltaTheta) + SS) * sin (DeltaTheta / 2);
      DeltaYSide = SideChord * cos (Theta + (DeltaTheta / 2));
      DeltaXSide = SideChord * sin (Theta + (DeltaTheta / 2));
      //DeltaXBack = BackChord * sin (Theta + (DeltaTheta / 2));
      //DeltaYBack = -BackChord * cos (Theta + (DeltaTheta / 2));
      Theta += DeltaTheta;
      X += DeltaXSide;
      Y += DeltaYSide;
    }

    //Odom heading is converting the radian value of Theta into degrees
    OdomHeading = Theta * 57.295779513;
    XPos = X/12;
    YPos = Y/12;
    //Converts values into newer values to allow for code to effectively work in next cycle
    PreviousL = currentL;
    PreviousR = currentR;
    DeltaTheta = 0;
    
    PreviousHeading = Heading;
    int textadjustvalue = 55;
    Brain.Screen.printAt(40,20 + textadjustvalue, "X-Pos:%f",XPos);
    Brain.Screen.setPenColor( vex::color( 22, 100, 8 ) );
    Brain.Screen.printAt(40,50 + textadjustvalue, "Y-Pos:%f",YPos);
    Brain.Screen.setPenColor( vex::color( 22, 100, 8 ) );
    Brain.Screen.printAt(40,80 + textadjustvalue, "Theta:%f",Theta);
    Brain.Screen.setPenColor( vex::color( 22, 100, 8 ) );
    Brain.Screen.printAt(40,110 + textadjustvalue, "Angle:%f",OdomHeading);
    vex::task::sleep(10);
    
  }
  return 1;
}