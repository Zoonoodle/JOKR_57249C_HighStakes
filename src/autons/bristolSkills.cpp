#include "lemlib/chassis/chassis.hpp"
#include "main.h"
#include "pros/rtos.hpp"
#include "sortingControl.h"
#include "robotConfigs.h"
#include "bennyHeaders/macros.h"
#include "bennyHeaders/DistanceSensorMovements.hpp"
#include "bennyHeaders/hardwareAndSensors.h"
#include <ctime>
#include <tuple>
void bristolSkills() {


std::int32_t set_integration_time(20);
          chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
          pros::Task sortingTask(sortingControlTask);
            

/////////////////SKILLS  START///////////////////

          chassis.setPose(0,0,180);
          twoRingChassis.setPose(0,0, 180);
          mogoChassis.setPose(0,0,180);
             setAutonState(1);
              pros::delay(150);
                  intake.move(127);
              pros::delay(300);
              intake.move(0);
              pros::delay(100);

              //backpack onto stake
              setAutonState(3);
              pros::delay(450);
              moveF(515, false, false, 60, 800); //was 520
              chassis.turnToHeading(270, 450);
               setAutonState(0);
               
              chassis.waitUntilDone();
             
              
              moveB(1336, false, true, 50, 800);
              // left_motors.move(-50);
              // right_motors.move(-50);
              
              left_motors.move(-40);
              right_motors.move(-40);
              pros::delay(150);
              mogo.set_value(true);
              pros::delay(100);
                left_motors.move(0);
              right_motors.move(0);


              //
              mogoChassis.turnToHeading(350, 400);
              mogoChassis.moveToPoint(16, 20, 400, {.earlyExitRange = 10});
              intake.move(127);
               mogoChassis.moveToPoint(34, 48, 600, {.earlyExitRange = 10});
           mogoChassis.moveToPoint(41, 77, 1000, {.maxSpeed = 100, .earlyExitRange = 10});





            mogoChassis.turnToHeading(25, 250);
           mogoChassis.waitUntilDone();
          
          moveFR(550, 148, true, true, true, 85, 900);
          intake.move(0);
          setAutonState(1);
          // moveFR(1690, 585, false, false, false, 80, 2000);
          twoRingChassis.moveToPoint(34, 55, 1700, {.forwards = false, .maxSpeed = 70});
          twoRingChassis.waitUntil(12);
          intake.move(127);
          twoRingChassis.turnToHeading(89, 650);
          // moveFR(1668, 535
          // , false, false, false, 80, 600);

          // moveR(544, false, false, 75,1300);
          //early into route so idk if we need distance


          twoRingChassis.waitUntilDone();
          currentArmState = 1; 
                  loadActivated   = true; 
                  targetArmState  = LoadStates[1]; 

          moveF(135, true, true, 80, 600);
          

          
          

          moveF(200, false, false, 80, 300);
          setAutonState(2);
          

          pros::delay(450);
          targetArmState = LoadStates[0];

            while (armSensor.get_position() > 12000 + 100) {
                  pros::delay(10);
                }
                
                pros::delay(150);
              
                intake.move(127);
                pros::delay(350); 
                intake.move(0);
                
                setAutonState(2);
                chassis.turnToHeading(90, 400);
                pros::delay(350);
                chassis.waitUntilDone();
          moveF(445, false, false, 70, 800);

          twoRingChassis.turnToHeading(180, 450, {.earlyExitRange = 1});
           setAutonState(0);
          
          intake.move(127);
         

         
          twoRingChassis.moveToPoint(48, 20, 500, {.earlyExitRange = 5});
          // chassis.moveToPoint(-14, 30, 500, {.earlyExitRange = 5});
           twoRingChassis.turnToHeading(148, 450);
          twoRingChassis.waitUntilDone();

          moveFL(160, 140, true, true ,true, 85, 800);

          // chassis.moveToPoint(-18, 46, 1000);

          twoRingChassis.moveToPoint(44, 31, 800, {.forwards = false});

          twoRingChassis.turnToHeading(180, 450);
          twoRingChassis.waitUntilDone();

          currentSortingCommand = STOPAFTER;
          moveF(190, true, true, 75, 1500);
          
          
          
          
          twoRingChassis.turnToHeading(290, 1500, {.maxSpeed = 60});
          twoRingChassis.waitUntilDone();
          right_motors.move(-65);
          left_motors.move(-65);
          intake.move(0);
          pros::delay(50);
          mogo.set_value(false);
          pros::delay(900);
  
          
          twoRingChassis.waitUntilDone();
           right_motors.move(0);
          left_motors.move(0);

          chassis.setPose(0,0,290);
        //   twoRingChassis.setPose(0,0, 292);
        //   mogoChassis.setPose(0,0,292);

        left_motors.move(127);
        right_motors.move(127);
        pros::delay(350);
          moveL(543, true, false, 50, 1000);

//           mogo.set_value(false);

 chassis.turnToHeading(85, 700);
 chassis.waitUntilDone();
 moveF(1980, false, false, 85, 1100);

   /////////////////////QUADRANT 2 ///////////////////////////////

              left_motors.move(-45);
              right_motors.move(-45);
              pros::delay(375);
              mogo.set_value(true);
              pros::delay(150);
                left_motors.move(0);
              right_motors.move(0);
 


mogoChassis.setPose(-4,0,90);

mogoChassis.turnToHeading(-350, 500, {.earlyExitRange =10});
    mogoChassis.moveToPoint(-12, 18, 400, {.earlyExitRange = 10});
    intake.move(127);
    mogoChassis.moveToPoint(-28, 48, 600, {.earlyExitRange = 10});
mogoChassis.moveToPoint(-35, 74, 700, {.maxSpeed = 100, .earlyExitRange = 10});

mogoChassis.turnToHeading(-35 , 350);
mogoChassis.waitUntil(5);

moveFL(425, 215, true, true, true, 80, 800); //43, 77
//36, 54


/*

*/

twoRingChassis.moveToPoint(-28, 42, 1200, {.forwards=  false, .earlyExitRange = 10});
twoRingChassis.turnToHeading(225, 500, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .earlyExitRange = 5});
twoRingChassis.moveToPoint(-36, 12, 700);
twoRingChassis.turnToHeading(205, 300);

twoRingChassis.waitUntilDone();
moveFR(540, 210,true, true, true, 85, 800);
 currentSortingCommand = STOPTHIRD;
twoRingChassis.moveToPoint(-32, 10 , 600, {.forwards = false});

twoRingChassis.turnToHeading(180, 500);

          moveF(260, true, true, 65, 1000);
         
         
          moveF(190, true, true, 60, 1500);
          
          
          twoRingChassis.turnToHeading(70, 900, {.maxSpeed = 100});
          
          twoRingChassis.waitUntilDone();
        
          right_motors.move(-65);
          left_motors.move(-65);
          pros::delay(100);
          mogo.set_value(false);
          pros::delay(700);
       
          
          twoRingChassis.waitUntilDone();





    right_motors.move(0);
          left_motors.move(0);
///////////////QUADRANT THREE ////////////////////////////////////
          chassis.setPose(0,0, 70);
          mogoChassis.setPose(0,0,70);
          twoRingChassis.setPose(0,0,70);
          
          left_motors.move(127);
          right_motors.move(127);
          pros::delay(100);
             right_motors.move(0);
          left_motors.move(0);
       chassis.moveToPoint(15, 48 , 800, {.earlyExitRange = 5});
setAutonState(1);
          chassis.waitUntil(20);
          intake.move(127);
          chassis.waitUntilDone();
          moveB(1845, true, false, 60, 1500);
          // moveL(592, true, false, 60, 1000);



          
              
         
          
          pros::delay(20);
          intake.move(127);
          chassis.turnToHeading(274, 600);
          chassis.waitUntilDone();

  currentArmState = 1; 
                  loadActivated   = true; 
                  targetArmState  = LoadStates[1]; 

          moveF(135, true, true, 70, 1000);
          

          
          

          moveF(200, false, false, 80, 300);
          setAutonState(2);
          

         
          pros::delay(450);
          targetArmState = LoadStates[0];

            while (armSensor.get_position() > 12000 + 100) {
                  pros::delay(10);
                }
                
                pros::delay(150);
              
                intake.move(127);
                pros::delay(350); 
                intake.move(0);
                
                setAutonState(2);
               
                pros::delay(400);
               
            //chassis.turnToHeading(271 , 450, {.minSpeed = 45});
                      chassis.waitUntilDone();
          moveF(500, false, false, 80, 700); //changed from 500

          chassis.turnToHeading(6, 600); //from 5, then 6

          
         
           intake.move(127);
          setAutonState(1);
          chassis.waitUntilDone();
          left_motors.move(80);
          right_motors.move(80);
          pros::delay(700);

 left_motors.move(60);
          right_motors.move(60);

          moveF(600, true, true, 55, 1000);
          setAutonState(1);
        



          ///////// QUADRANT 4 ?//////
      
          chassis.resetLocalPosition();
          chassis.moveToPoint(0, -6, 350, {.forwards= false});

        intake.move(127);

        chassis.turnToHeading(-110, 600);
        
        left_motors.set_brake_mode(pros::MotorBrake::coast);
          right_motors.set_brake_mode(pros::MotorBrake::coast);
        chassis.moveToPoint(22, 11, 1500, {.forwards= false, .maxSpeed = 60});
          chassis.waitUntil(30);
          mogo.set_value(true);

             //RSENSOR: 366, then 165 after turn (front should be 1122)
          

          
          
          


//           ///////////////QUADRANT 3.5 ?/////////////////////////
//           mogoChassis.setPose(0,0, -123);
          intake.move(127);
          // left_motors.move(-45);
          // right_motors.move(-45);
          // pros::delay(150);
          // mogo.set_value(true);
          // pros::delay(150);

          
//           left_motors.move(0);
//           right_motors.move(0);
//           mogo.set_value(true);
          chassis.waitUntilDone();

          left_motors.set_brake_mode(pros::MotorBrake::hold);
          right_motors.set_brake_mode(pros::MotorBrake::hold);
          mogoChassis.turnToHeading(295, 600);
            pros::delay(250);
            currentArmState = 1; 
                  loadActivated   = true; 
                  targetArmState  = LoadStates[1]; 



               doinker.set_value(true);      
   mogoChassis.waitUntilDone();

   mogoChassis.moveToPoint(-8, 15, 1500,{.maxSpeed = 80});
  
 
mogoChassis.turnToHeading(215, 800, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .minSpeed = 75});
mogoChassis.turnToHeading(155, 650);

  //  mogoChassis.turnToHeading(230, 450, {.maxSpeed = 85,.earlyExitRange = 10});
  //   mogoChassis.turnToHeading(280, 400);
      // moveF(500, true, true, 60, 1000);
      doinker.set_value(false);
      // mogoChassis.moveToPose(-3, 0, 139, 1500, {.maxSpeed = 70});

      
  //  mogoChassis.turnToHeading(138, 1000);




   mogoChassis.waitUntilDone();
    mogo.set_value(false);
   pros::delay(20);
   left_motors.move(-60);
   right_motors.move(-60);
   pros::delay(900);
// left_motors.move(-25);
//    right_motors.move(-25);
   
      left_motors.move(0);
   right_motors.move(0);

// chassis.setPose(0,0, 130);
chassis.setPose(0,0,130);
mogoChassis.setPose(0,0,130);
twoRingChassis.setPose(0,0,130);

   pros::delay(20);
   left_motors.move(120);
   right_motors.move(120);
   pros::delay(200);
         left_motors.move(0);
   right_motors.move(0);


moveL( 627, true, false, 70, 1000);

chassis.turnToHeading(270, 600);
chassis.waitUntilDone();

chassis.setPose(0,0,0);
mogoChassis.setPose(0,0,0);
twoRingChassis.setPose(0,0,0);


chassis.moveToPoint(-0.5, -41, 1500, {.forwards = false, .maxSpeed = 60,.earlyExitRange = 3 });
chassis.waitUntil(39);
mogo.set_value(true);
chassis.waitUntilDone();
moveF(1690, false, false, 55, 700);
mogoChassis.turnToHeading(88, 600);
mogoChassis.waitUntilDone();
moveF(345, true, true, 70, 800);
setAutonState(3);
intake.move(127);
pros::delay(450);
mogoChassis.turnToHeading(91, 200);
mogoChassis.waitUntilDone();

moveF(560, false, false, 80, 1000);
setAutonState(0);
mogoChassis.moveToPoint(-24, -70,1200);
mogoChassis.turnToHeading(135, 600);
mogoChassis.moveToPoint(1, -74, 1500, {.maxSpeed = 70, .earlyExitRange = 5});


mogoChassis.moveToPose(15, -78, 75, 2000);

   // // mogoChassis.swingToHeading(145, lemlib::DriveSide::LEFT, 1000, {.minSpeed = 40});


//     // mogoChassis.turnToHeading(15, 650, {.direction = lemlib::AngularDirection::CW_CLOCKWISE});
//     mogoChassis.waitUntilDone();
//     mogo.set_value(false);
//     pros::delay(100);
//     moveBR(435, 285, false, true, true, 55, 800);
    

//     left_motors.move(40);
//     right_motors.move(40);
//     pros::delay(500);
//      left_motors.move(-55);
//     right_motors.move(-55);
//     pros::delay(500);
//        left_motors.move(0);
//     right_motors.move(0);

    
    
    
//     mogoChassis.moveToPoint(-5,-8, 800);
//     mogoChassis.turnToHeading(75, 600);

    



//     pros::delay(9999999);


//     // moveBL(1422, 430, true, false, false, 90, 1500);
//     mogoChassis.turnToHeading(280, 700);
//     mogoChassis.waitUntilDone();

//  left_motors.set_brake_mode(pros::MotorBrake::coast);
//           right_motors.set_brake_mode(pros::MotorBrake::coast);


//           moveF(1450, false, false, 75, 800);
//           // moveF(1700, false, false, 55, 900);
//           left_motors.move(-45);
//           right_motors.move(-45);
//           pros::delay(220);
//           mogo.set_value(true);
//           pros::delay(150);

          
//           left_motors.move(0);
//           right_motors.move(0);
//           mogo.set_value(true);
//           mogoChassis.turnToHeading(10, 650);
//           mogoChassis.waitUntilDone();
//           moveF(342, true, true, 60, 800);
//           setAutonState(3);
//           pros::delay(400);
//           intake.move(127);
//           pros::delay(550);
//           moveF(550, false, false, 80, 800);
//           setAutonState(0);
//           mogoChassis.turnToHeading(240, 600); 
//           mogoChassis.moveToPoint(-26, -38, 1000);


        //RSENSOR: 366, then 165 ater turn (front should be 1122)


    //  mogo.set_value(false);
    //  pros::delay(500);
    // moveB(820, true,  false, 80, 1000);
  
  //  mogoChassis.turnToHeading(295, 700);
   

  //  moveFR(220, 40, true, true, true, 100, 1500);
}
