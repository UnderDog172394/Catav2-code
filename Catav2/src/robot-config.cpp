#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor L1 = motor(PORT2, ratio6_1, true);
motor L2 = motor(PORT3, ratio6_1, false);
motor L3 = motor(PORT4, ratio6_1, true);
motor R1 = motor(PORT5, ratio6_1, false);
motor R2 = motor(PORT6, ratio6_1, true);
motor R3 = motor(PORT7, ratio6_1, false);
inertial Head = inertial(PORT18);
rotation LTrack = rotation(PORT20, true);
rotation RTrack = rotation(PORT19, false);
digital_out Single = digital_out(Brain.ThreeWirePort.A);
digital_out Double = digital_out(Brain.ThreeWirePort.B);
motor Cata = motor(PORT8, ratio36_1, true);
rotation CataArm = rotation(PORT9, false);
motor Intake = motor(PORT11, ratio18_1, false);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}