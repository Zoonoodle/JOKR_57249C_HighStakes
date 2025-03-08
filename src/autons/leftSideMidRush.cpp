#include "lemlib/chassis/chassis.hpp"
#include "main.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "sortingControl.h"
#include "robotConfigs.h"
#include "bennyHeaders/macros.h"
#include "bennyHeaders/DistanceSensorMovements.hpp"
#include "bennyHeaders/hardwareAndSensors.h"
#include "bennyHeaders/autons.h"
#include <ctime>



void leftSideMidRush() {

    pros::Task sortingTask(sortingControlTask);
   
    std::int32_t set_integration_time(20);
    chassis.setPose(0,0,0);
    mogoChassis.setPose(0,0,0);

    setAutonState(1);
    chassis.turnToHeading(325, 400);
        pros::delay(200);
        intake.move(127);
        pros::delay(350);
        intake.move(0);
        pros::delay(40);
        setAutonState(3);
        moveF(445, true, true, 50, 600);
        
        pros::delay(300);
      
        
        chassis.moveToPoint(12, -30, 1750, {.forwards = false, .maxSpeed = 70});
        //-24, 12-30
        chassis.waitUntil(33);
        setAutonState(0);
        mogo.set_value(true);//23-42-251
        pros::delay(200);
        intake.move(0);
        setAutonState(0);

        // chassis.waitUntilDone();
        // mogoChassis.turnToPoint(-14, -34, 650);
        // mogoChassis.waitUntilDone();
        // doinker2.set_value(true);
        // mogoChassis.moveToPoint(-21, -39, 1200, {.maxSpeed = 80});
        // mogoChassis.moveToPoint(15, -27, 1000, {.forwards = false});
        // mogoChassis.waitUntilDone();
        // doinker2.set_value(false);

        // pros::delay(100);
        // intake.move(127);
        // mogoChassis.moveToPoint(3, -35, 800);
        // mogoChassis.turnToHeading(-245, 650);

        // mogoChassis.moveToPoint(19, -42, 800, {.earlyExitRange = 5});
        // mogoChassis.turnToHeading(-301, 350);
        // mogoChassis.moveToPoint(35, -20, 800);
        // mogoChassis.turnToHeading(-275, 750, {.earlyExitRange = 20});
        // mogoChassis.moveToPoint(60, -23, 1000,{.maxSpeed = 80});
        // mogoChassis.moveToPoint(40, -23, 1000, {.forwards = false});
        // // doinker2.set_value(true);

        // mogoChassis.moveToPoint(-15, -39, 1200);
        // mogoChassis.moveToPoint(12, -28, 1000, {.forwards = false});

        // mogoChassis.waitUntilDone();
        // doinker2.set_value(false);
        // pros::delay(100);
        
        // mogoChassis.moveToPoint(0, -26, 600);
        // mogoChassis.turnToHeading(-122, 650);
        // mogoChassis.moveToPoint(-18, -35, 1200, {.maxSpeed = 80});
        // doinker2.set_value(true);

        // mogoChassis.turnToHeading(-140, 500);
        // mogoChassis.moveToPoint(-19, -37, 300);
        // doinker.set_value(true);

}

/*

 pros::Task sortingTask(sortingControlTask);

    std::int32_t set_integration_time(20);
    sorter.set_led_pwm(80);
    mogoChassis.setPose(0,0,0);
    chassis.setPose(0,0,0);
    
        setAutonState(1);
        pros::delay(500);
        chassis.setPose(0,0,0);
        intake.move(127);
        
        pros::delay(350);
        intake.move(0);
        pros::delay(40);
        setAutonState(3);
       
        pros::delay(500);
        
        chassis.moveToPoint(-2, -26 , 1300, {.forwards = false});
        chassis.waitUntil(20);
        setAutonState(0);
        chassis.turnToHeading(-275, 400);
        
         chassis.moveToPoint(-30, -30, 1000, {.forwards = false, .maxSpeed = 60});
        chassis.waitUntil(22);
        mogo.set_value(true);
        chassis.waitUntilDone();
    
        
    
        mogoChassis.turnToHeading(-70, 750);
    
    
        mogoChassis.waitUntilDone();
        chassis.waitUntilDone();
        mogoChassis.setPose(0,0,0);
        chassis.setPose(0,0,0);
    
        mogoChassis.moveToPoint(3, 12, 550, {.maxSpeed = 70});
        // mogoChassis.turnToHeading(-10, 250);
       
       
        pros::delay(300);
        mogoChassis.turnToHeading(-5, 250);
        mogoChassis.waitUntilDone();
        doinker2.set_value(true);
        mogoChassis.turnToHeading(-28, 300);
        mogoChassis.moveToPoint(-4 , 13,600);
        mogoChassis.turnToHeading(chassis.getPose().theta - 10, 250);
        mogoChassis.waitUntilDone();
        doinker.set_value(true);
        pros::delay(300);
        mogoChassis.moveToPoint(17, -24, 1000, {.forwards = false, .earlyExitRange = 5});
        
        mogoChassis.waitUntilDone();
        doinker.set_value(false);
            doinker2.set_value(false);
    
            pros::delay(300);
           
            intake.move(127);
            mogoChassis.moveToPoint(18, -10, 500, {.earlyExitRange = 10});
            mogoChassis.turnToHeading(-85,300, {.earlyExitRange = 10});
            mogoChassis.moveToPoint(-13, -25, 1000);
            mogoChassis.waitUntilDone();
            mogoChassis.turnToHeading(-200, 550);
            mogoChassis.moveToPoint(-2, -57, 1000);
            mogoChassis.waitUntil(10);
            doinker.set_value(true);
            mogoChassis.waitUntilDone();
           
           
    
            mogoChassis.turnToHeading(-320, 800, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 90});
          
            mogoChassis.moveToPoint(13, -50, 700, {.earlyExitRange = 10});
            doinker.set_value(false);
            currentIntakeCommand = unclampAfterOne;

            
            mogoChassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
            mogoChassis.moveToPoint(-24, -30,600, {.earlyExitRange = 5});
   
            setAutonState(0);

*/