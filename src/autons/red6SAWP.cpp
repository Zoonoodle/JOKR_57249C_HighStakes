#include "lemlib/chassis/chassis.hpp"
#include "main.h"
#include "pros/motors.h"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include "sortingControl.h"
#include "robotConfigs.h"
#include "bennyHeaders/macros.h"
#include "bennyHeaders/DistanceSensorMovements.hpp"
#include "bennyHeaders/hardwareAndSensors.h"
#include "bennyHeaders/autons.h"
#include <ctime>

void red6SAWP() {

    pros::Task sortingTask(sortingControlTask);
   
    std::int32_t set_integration_time(20);
    chassis.setPose(0,0,0);
    mogoChassis.setPose(0,0,0);

    setAutonState(1);
    chassis.turnToHeading(325, 400);
        pros::delay(400);//200
        intake.move(127);
        pros::delay(200); //350
        //temp
        intake.move(0);
        pros::delay(50);
        intake.move(127);
        pros::delay(50);
        // end temp
        intake.move(0);
        pros::delay(40);
        setAutonState(3);
        //260,340
        moveDualFront(340, 270, true, true, true, 70, 300);
        
        pros::delay(300);
      
        
        chassis.moveToPoint(13, -30, 1750, {.forwards = false, .maxSpeed = 70});
        //-24, 12-30
        chassis.waitUntil(35);
        setAutonState(0);
        mogo.set_value(true);//23-42-251
        pros::delay(200);
        chassis.turnToHeading(135, 650);
        intake.move(127);

        //middle rush
        chassis.moveToPoint(24, -40, 900, {.minSpeed = 30, .earlyExitRange = 3});
        chassis.turnToHeading(100, 400, {.minSpeed = 60});
        chassis.moveToPoint(38, -42, 750, {.maxSpeed = 60});
        chassis.waitUntilDone();
        chassis.moveToPoint(15, -34, 800, {.forwards = false, .earlyExitRange = 5});
        chassis.turnToHeading(65, 400);
        chassis.moveToPoint(30, -27, 800);
        chassis.turnToHeading(305, 750);
        chassis.moveToPoint(6, -10, 1000, {.minSpeed = 60, .earlyExitRange = 7});
        
        chassis.moveToPoint(-18, 0, 800, {.maxSpeed = 60});
        intakeLift.set_value(true);
        mogo.set_value(false);
        chassis.turnToHeading(20, 300);
        intakeLift.set_value(false);
        intake.move(0);
        pros::delay(600);
        intake.move(80);
        chassis.waitUntilDone();
        intake.move(0);
        chassis.moveToPoint(-30, -26, 1500, {.forwards = false, .maxSpeed = 50});
        chassis.waitUntil(36);
        mogo.set_value(true);
        pros::c::delay(100);
        intake.move(127);
        chassis.waitUntilDone();
        chassis.turnToHeading(270, 750);
        chassis.moveToPoint(-45, -26, 1000, {.minSpeed = 60});
        chassis.moveToPoint(-32, -26, 1000, {.forwards = false, .minSpeed = 50});
        chassis.waitUntilDone();
        chassis.turnToHeading(135, 500);
        pros::delay(100);
        setAutonState(2);
        
}