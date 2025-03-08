#include "lemlib/chassis/chassis.hpp"
#include "main.h"
#include "pros/abstract_motor.hpp"
#include "pros/rtos.hpp"
#include "sortingControl.h"
#include "robotConfigs.h"
#include "bennyHeaders/macros.h"
#include "bennyHeaders/DistanceSensorMovements.hpp"
#include "bennyHeaders/hardwareAndSensors.h"
#include <ctime>

void coenSkills(){
    mogoChassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

    mogoChassis.setPose(0,0,0);

    //onto stake
    intake.move(127);
    pros::delay(400);
    intake.move(0);

    //go to mogo
    mogoChassis.moveToPoint(0, 14, 750);
    mogoChassis.waitUntilDone();
    mogoChassis.turnToHeading(270, 900);
    mogoChassis.moveToPoint(22, mogoChassis.getPose().y, 1000, {.forwards = false, .maxSpeed = 80});
    mogoChassis.waitUntil(19);
    mogo.set_value(true);

    //get first mogo filled
    mogoChassis.turnToHeading(0, 750);
    mogoChassis.moveToPoint(20, 34, 800, {.minSpeed = 60, .earlyExitRange = 4});
    intake.move(127);
    mogoChassis.moveToPoint(45, 59, 750);
    mogoChassis.turnToHeading(90, 750);
  
    mogoChassis.moveToPoint(55, 58, 750);
   
    mogoChassis.moveToPoint(44, 58, 750, {.forwards = false});

    //grab the rings in the corner
    mogoChassis.turnToHeading(180, 650);
    mogoChassis.moveToPoint(46, 35, 900);
    mogoChassis.turnToHeading(135, 750);
    mogoChassis.moveToPoint(57, 18, 1000);
    mogoChassis.moveToPoint(46, 35, 1000, {.forwards = false});
    mogoChassis.turnToHeading(180, 750);
    mogoChassis.moveToPoint(48, -2, 1500, {.maxSpeed = 50});
    mogoChassis.moveToPoint(48, 19, 750, {.forwards = false});
    mogoChassis.turnToHeading(315, 750);
    mogoChassis.moveToPoint(61, 1, 1000, {.forwards = false, .maxSpeed = 60});

    mogo.set_value(false);
    mogoChassis.moveToPoint(-2, 9, 2750);
    mogoChassis.waitUntilDone();
    mogoChassis.turnToHeading(80, 1000);
    mogoChassis.moveToPoint(-19, 9, 800, {.forwards = false, .minSpeed = 60, .earlyExitRange = 5});
    mogoChassis.moveToPoint(-24, 9, 1000, {.forwards = false, .maxSpeed = 50});
    mogoChassis.waitUntil(4);
    mogo.set_value(true);
    


    //MOGO 2
    //get first mogo filled
    mogoChassis.turnToHeading(0, 750);
    mogoChassis.moveToPoint(-20, 32, 1000, {.minSpeed = 60, .earlyExitRange = 4});
    intake.move(127);
    mogoChassis.turnToPoint(-42, 52, 650);
    mogoChassis.moveToPoint(-42, 52, 1000);
    mogoChassis.turnToHeading(270, 750);
    mogoChassis.moveToPoint(-57, 52, 750);

    mogoChassis.moveToPoint(-43, 52, 750, {.forwards = false});
    
    mogoChassis.turnToHeading(180, 750);
    mogoChassis.moveToPoint(-43, 30, 800);
    mogoChassis.turnToPoint(-56, 10, 500);
    mogoChassis.moveToPoint(-56, 10, 900);
    mogoChassis.moveToPoint(-43, 27, 800, {.forwards = false});
    mogoChassis.turnToHeading(180, 500);
    mogoChassis.moveToPoint(-48, -5, 1500, {.maxSpeed = 50});
    mogoChassis.moveToPoint(-48, 8, 800, {.forwards = false});
    mogoChassis.turnToHeading(45, 750);
    mogoChassis.moveToPoint(-54, -10, 1000, {.forwards = false, .maxSpeed = 60});
    intake.move(0);
    mogo.set_value(false);
    mogoChassis.moveToPoint(-42, 73, 6000, {.maxSpeed = 90});
    mogoChassis.waitUntil(40);
    intake.move(100);
    setAutonState(1);
    mogoChassis.waitUntilDone();
    pros::delay(1000);
    mogoChassis.moveToPoint(-43, 96, 1000, {.maxSpeed = 60});
    currentArmState = 1; 
    loadActivated   = true; 
    targetArmState  = LoadStates[1]; 
  
 
    mogoChassis.turnToHeading(267, 650);
    intake.move(127);
    mogoChassis.moveToPoint(0, 96, 2000, {.forwards = false, .maxSpeed = 70});
    intake.move(0);
    setAutonState(1);
    mogoChassis.waitUntil(40);
    mogo.set_value(true);
    mogoChassis.turnToHeading(0, 650);
    mogoChassis.moveToPoint(0, 115, 2000, {.maxSpeed = 75});
    mogoChassis.waitUntilDone();
    loadActivated   = false; 
    moveF(324, false, false, 60, 1500);
    setAutonState(3);
    pros::delay(200);
    intake.move(127);
    pros::delay(300);
    moveF(500, false, false, 60, 750);
    mogoChassis.turnToHeading(225, 750);
    setAutonState(0);
    mogoChassis.moveToPoint(-23, 70, 1200);
    intake.move(4);
    mogoChassis.turnToHeading(135, 750);
    mogoChassis.waitUntilDone();
    pros::delay(100);
    mogoChassis.moveToPoint(0, 50, 1000);
    mogoChassis.turnToHeading(45, 750);
    intake.move(127);
    mogoChassis.waitUntilDone();
    pros::delay(500);
    mogoChassis.moveToPoint(20, 70, 1200);
    intake.move(0);
    mogoChassis.waitUntilDone();
    intake.move(127);
    mogoChassis.turnToHeading(90, 650);
    mogoChassis.moveToPoint(45, 68, 1000);
    mogoChassis.turnToHeading(45, 650);
    doinker.set_value(true);
    mogoChassis.moveToPoint(60, 98, 1500);
    intake.move(127);
    mogoChassis.turnToHeading(190, 1000, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE});
    mogoChassis.waitUntilDone();
    intake.move(0);
    mogoChassis.moveToPoint(63, 110, 1000, {.forwards = false});
    mogo.set_value(false);
 }