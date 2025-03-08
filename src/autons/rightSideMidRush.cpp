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
#include <tuple>

void rightSideMidRush() {
             

pros::Task intakeTawsk(intakeControlTask);

std::int32_t set_integration_time(20);
sorter.set_led_pwm(80);
mogoChassis.setPose(0,0,0);
chassis.setPose(0,0,0);
pros::Task sortingTask(sortingControlTask);

//load for Alliance Stake

setAutonState(1);

// turn and Score on alliance
chassis.turnToHeading(-325, 400);
    pros::delay(300);
    intake.move(127);
    pros::delay(150);
    intake.move(0);
    pros::delay(40);
    setAutonState(3);
   moveDualFront(265, 339, true, true, true, 60, 500);
    
    pros::delay(200);
  
    

    // move to mogo
    chassis.moveToPoint(-13, -30, 1750, {.forwards = false, .maxSpeed = 70});
    chassis.waitUntil(33);
    setAutonState(0);

    // clamp mogo
    mogo.set_value(true);//23-42-251
    intake.move(40);
    pros::delay(200);
    intake.move(0);
    setAutonState(0);

    //turn to middle ring
    chassis.turnToHeading(115, 450);

    //grab first middle
    chassis.moveToPoint(1, -38, 900);
    doinker.set_value(true);
    

    //swing to second
    chassis.swingToHeading(155, lemlib::DriveSide::RIGHT, 450, {.minSpeed = 30});
    
    pros::delay(300);
    doinker2.set_value(true);
    chassis.waitUntilDone();
    left_motors.move(20);
    right_motors.move(20);
    pros::delay(150);
    left_motors.move(0);
    right_motors.move(0);



    chassis.moveToPoint(-23, -3, 800, {.forwards = false});
    chassis.waitUntilDone();
    doinker.set_value(false);
    doinker2.set_value(false);
    pros::delay(100);
    intake.move(127);
    
    chassis.moveToPoint(-12, -12, 550, {.earlyExitRange = 10});
    chassis.swingToHeading(240, lemlib::DriveSide::RIGHT, 400, {.earlyExitRange = 10});
    chassis.moveToPoint(-40, -24, 1250);



    chassis.turnToHeading(330, 550);
    chassis.moveToPoint(-46, -1, 700, {.earlyExitRange = 8});




   chassis.swingToHeading(310, lemlib::DriveSide::LEFT, 550);
  
//    intakeLift.set_value(false);
//    pros::delay(100);
 chassis.moveToPoint(-51,6, 1200, {.minSpeed = 90});
 chassis.moveToPoint(-49, 3.5, 700, {.forwards = false, .maxSpeed = 60, .minSpeed =60, .earlyExitRange = 1});
 chassis.moveToPoint(-51,6, 900, {.minSpeed = 90});
intakeLift.set_value(true);
    pros::delay(400);
    intakeLift.set_value(false);
    
    chassis.moveToPoint(-39, -13, 700, {.forwards = false, .earlyExitRange = 10});

 
    setAutonState(1);
    chassis.turnToHeading(140, 500, {.earlyExitRange = 10});
    currentIntakeCommand = STOPRED;
    chassis.moveToPoint(-43, -40, 1000);
    intake.move(127);
    mogo.set_value(false);

    chassis.turnToHeading(235, 650);
    currentArmState = 1; 
    loadActivated   = true; 
    targetArmState  = LoadStates[1]; 
    setAutonState(7);
    moveDualFront(285, 168, true, true, true, 80, 1000);
    

    
//     mogoChassis.turnToHeading(70, 750);


//     mogoChassis.waitUntilDone();
//     chassis.waitUntilDone();
//     mogoChassis.setPose(0,0,0);
//     chassis.setPose(0,0,0);
// intake.move(15);

//     mogoChassis.moveToPoint(-3, 12.5, 550, {.maxSpeed = 70});
//     mogoChassis.turnToHeading(8, 250);
//    mogoChassis.waitUntilDone();
//     doinker.set_value(true);
//     pros::delay(220);
//     mogoChassis.turnToHeading(27, 350);

//     mogoChassis.moveToPoint(4, 13.5, 800);
//     mogoChassis.waitUntilDone();
//     doinker2.set_value(true);
//     pros::delay(300);
//     mogoChassis.moveToPoint(-17, -24, 1000, {.forwards = false, .earlyExitRange = 5});
    
//     mogoChassis.waitUntilDone();
//     doinker.set_value(false);
//         doinker2.set_value(false);

//         pros::delay(300);
//         mogoChassis.turnToHeading(-5, 350);
//         intake.move(127);
//         mogoChassis.moveToPoint(-17, -12, 600, {.earlyExitRange = 10});
//         mogoChassis.turnToHeading(85, 200, {.earlyExitRange = 10});
//         mogoChassis.moveToPoint(11, -25, 1000);
//         mogoChassis.waitUntilDone();
//         mogoChassis.turnToHeading(195, 350);
//         mogoChassis.moveToPoint(1, -56, 1000);
//         mogoChassis.waitUntil(10);
//         doinker2.set_value(true);
//         mogoChassis.waitUntilDone();
       
       

//         mogoChassis.turnToHeading(315, 800, {.direction = lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 90});
//         mogoChassis.moveToPoint(-12, -50, 500, {.earlyExitRange = 10});
//         // currentIntakeCommand = unclampAfterOne;
//         doinker2.set_value(false);
       
//         mogoChassis.moveToPoint(18, 0,1000);
       
        // mogoChassis.turnToHeading(355, 550);
        // mogoChassis.waitUntil(12);
      
        //turn 195
        //move -57, 19
       

//     pros::delay(10000);
//     mogoChassis.moveToPoint(41, -20, 1000, {.maxSpeed = 70});

//     mogoChassis.turnToHeading(85, 250);
//     mogoChassis.waitUntil(13);

//     doinker.set_value(true);
//     pros::delay(100);
//     mogoChassis.swingToHeading(106, lemlib::DriveSide::RIGHT, 500);
//     mogoChassis.waitUntilDone();
//     doinker2.set_value(true);
//     pros::delay(350);





//    mogoChassis.moveToPoint(35, -28, 600, {.forwards = false, .earlyExitRange = 15});
//    mogoChassis.moveToPoint(4, -36, 1000, {.forwards = false});

//     mogoChassis.waitUntilDone();
//     

//     pros::delay(250);
//     mogoChassis.turnToHeading(60, 400);
//     intake.move(127);
   
//     mogoChassis.moveToPoint(13.6, -27.6, 600);

//     mogoChassis.swingToHeading(245, lemlib::DriveSide::RIGHT, 700, {.direction = lemlib::AngularDirection::CW_CLOCKWISE});
//     mogoChassis.moveToPoint(-4, -53, 1000, {.maxSpeed = 80});

//     mogoChassis.turnToHeading(315, 400);
    
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
    currentIntakeCommand = STOPFIRST;


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
            
            


    }


    /*
    
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
    currentIntakeCommand = STOPFIRST;


    chassis.moveToPoint(-27, 30, 1300);
    intake.move(127);
    
    
    chassis.turnToHeading(270, 800);
    
    
    
    
    chassis.moveToPoint(-4, 30, 1500, {.forwards = false, .maxSpeed = 53});
    chassis.waitUntil(21);
    mogo.set_value(true);
    intake.move(127);
    */