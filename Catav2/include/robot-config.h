using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor L1;
extern motor L2;
extern motor L3;
extern motor R1;
extern motor R2;
extern motor R3;
extern inertial Head;
extern rotation LTrack;
extern rotation RTrack;
extern digital_out Single;
extern digital_out Double;
extern motor Cata;
extern rotation CataArm;
extern motor Intake;
extern optical roller;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );