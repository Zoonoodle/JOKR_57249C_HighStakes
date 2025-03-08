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
    sorter.set_led_pwm(80);
    mogoChassis.setPose(0,0,0);
    chassis.setPose(0,0,0);
    
    //load for Alliance Stake
    
    setAutonState(1);
    
    // turn and Score on alliance
    chassis.turnToHeading(324, 400);
        pros::delay(300);
        intake.move(127);
        pros::delay(150);
        intake.move(0);
        pros::delay(40);
        setAutonState(3);
       moveDualFront(339, 260, true, true, true, 60, 500);
        
        pros::delay(200);
      
        
        chassis.moveToPoint(12, -30, 1750, {.forwards = false, .maxSpeed = 70});
        //-24, 12-30
        chassis.waitUntil(33);
        setAutonState(0);
        mogo.set_value(true);//23-42-251
        pros::delay(200);
        chassis.turnToHeading(135, 650);
        intake.move(127);
        chassis.moveToPoint(24, -40, 900, {.minSpeed = 30, .earlyExitRange = 3});
        chassis.turnToHeading(100, 400, {.minSpeed = 60, .earlyExitRange = 20});
        chassis.moveToPoint(37, -42, 750, {.maxSpeed = 60});
        chassis.waitUntilDone();
        chassis.moveToPoint(15, -34, 800, {.forwards = false, .earlyExitRange = 5});
        chassis.turnToHeading(65, 400);
        chassis.moveToPoint(30, -27, 1000);
        chassis.turnToHeading(305, 750);
        chassis.moveToPoint(6, -10, 1000, {.minSpeed = 60, .earlyExitRange = 7});
        intakeLift.set_value(true);
        chassis.moveToPoint(-18, 0, 800, {.maxSpeed = 65});
        chassis.waitUntilDone();
        mogo.set_value(false);
        chassis.turnToHeading(20, 750);
        intake.move(0);
        chassis.moveToPoint(-30, -21, 1500, {.forwards = false, .maxSpeed = 50});
        chassis.waitUntilDone();
        mogo.set_value(true);
        intake.move(127);

}