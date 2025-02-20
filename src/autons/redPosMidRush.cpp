#include "lemlib/chassis/chassis.hpp"
#include "main.h"
#include "pros/rtos.hpp"
#include "sortingControl.h"
#include "robotConfigs.h"
#include "bennyHeaders/macros.h"
#include "bennyHeaders/DistanceSensorMovements.hpp"
#include "bennyHeaders/hardwareAndSensors.h"
#include "bennyHeaders/autons.h"
#include <ctime>
#include <tuple>

void redPosMidRush() {
             
pros::Task sortingTask(sortingControlTask);

std::int32_t set_integration_time(20);

    setAutonState(1);
    pros::delay(150);
    chassis.setPose(0,0,0);
    intake.move(127);
    
    pros::delay(520);
    intake.move(0);
    pros::delay(200);
    setAutonState(3);
    setAutonState(3);
    pros::delay(500);
    
    chassis.moveToPoint(0, -14.5, 700, {.forwards = false, .maxSpeed = 70});
    pros::delay(200);
    setAutonState(0);
    chassis.waitUntilDone();
    chassis.setPose(0,0,240);
    chassis.turnToPoint(24, 30, 700);
    chassis.waitUntilDone();
    currentSortingCommand = STOPFIRST;


    chassis.moveToPoint(20, 27, 2000, {.maxSpeed = 80});
    intake.move(110);
    
    
    chassis.turnToHeading(-270, 800);
    
    
    
    
    chassis.moveToPoint(-1, 28, 2000, {.forwards = false, .maxSpeed = 55});
    chassis.waitUntil(24);
    mogo.set_value(true);
    pros::delay(200);
    intake.move(127);      
    chassis.turnToHeading(50, 500);
    chassis.moveToPose(23, 43, 90, 2000, {.maxSpeed = 75});

chassis.moveToPoint(10, 4, 2000, {.forwards = false, .maxSpeed = 80});
chassis.turnToHeading(270, 800);
currentSortingCommand = SORTFIRST;
moveF(400, true, true, 80, 3000);

   




}


/*

pros::delay(4000);
    setAutonState(1);
    pros::delay(100);
    chassis.setPose(0,0,0);
    intake.move(127);
    
    pros::delay(520);
    intake.move(0);
    pros::delay(200);
    setAutonState(3);
    setAutonState(3);
    pros::delay(500);
    
    chassis.moveToPoint(0, -14.5, 700, {.forwards = false, .maxSpeed = 70});
    pros::delay(200);
    setAutonState(0);
    chassis.waitUntilDone();
    chassis.setPose(0,0,240);
    chassis.turnToPoint(24, 30, 700);
    currentSortingCommand = STOPFIRST;


    chassis.moveToPoint(20, 27, 2000, {.maxSpeed = 80});
    intake.move(110);
    
    
    chassis.turnToHeading(-270, 800);
    
    
    
    
    chassis.moveToPoint(-1, 28, 2000, {.forwards = false, .maxSpeed = 55});
    chassis.waitUntil(24);
    mogo.set_value(true);
    pros::delay(200);
    intake.move(127);
    chassis.turnToHeading(45, 600);
    chassis.waitUntilDone();
    chassis.setPose(0,0,0);
    chassis.moveToPoint(-4, 22, 2000);
    chassis.moveToPoint(5, 0, 1000, {.forwards = false});
*/



    void BluePosRedNeg() {

    
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
          pros::Task sortingTask(sortingControlTask);
            

             setAutonState(1);
              pros::delay(220);
                  intake.move(127);
              pros::delay(400);
              intake.move(0);
              pros::delay(100);

    chassis.setPose(0,0,0);
    setAutonState(3);
    pros::delay(500);
    
    chassis.moveToPoint(0, -14.5, 700, {.forwards = false, .maxSpeed = 70});
    pros::delay(200);
    setAutonState(0);
    chassis.waitUntilDone();
    chassis.setPose(0,0,120);
    chassis.turnToPoint(-26, 30, 700);
    currentSortingCommand = STOPFIRST;


    chassis.moveToPoint(-27, 30, 1300);
    intake.move(127);
    
    
    chassis.turnToHeading(270, 800);
    
    
    
    
    chassis.moveToPoint(-4, 30, 1500, {.forwards = false, .maxSpeed = 53});
    chassis.waitUntil(21);
    mogo.set_value(true);
    intake.move(127);

    }