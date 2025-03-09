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
#include <tuple>
void statesSkills() {


std::int32_t set_integration_time(20);
          chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
          pros::Task intakeTask(intakeControlTask);
            

//           chassis.turnToHeading(90, 1000);
//           chassis.turnToHeading(0, 1000);
// /////////////////SKILLS  START///////////////////
//    pros::delay(9999);
          chassis.setPose(0,0,180);
          twoRingChassis.setPose(0,0, 180);
          mogoChassis.setPose(0,0,180);
             setAutonState(1);
             pros::delay(300);
             intake.move(127);
             pros::delay(200);
             intake.move(0);
             pros::delay(40);
             setAutonState(3);

              //backpack onto stake
              setAutonState(3);
              pros::delay(450);
              // chassis.moveToPoint(0, 7, 900, {.forwards = false});
              // moveF(500, false, false, 60, 800); //was 520
              chassis.moveToPoint(0, 7, 600, {.forwards = false});
              chassis.turnToHeading(269, 650);
               setAutonState(0);
               
              chassis.waitUntilDone();
            //  chassis.moveToPoint(0, -6, 1000);
            chassis.moveToPoint(23,chassis.getPose().y, 2000, {.forwards = false, .maxSpeed = 60});
            chassis.waitUntil(18);
            mogo.set_value(true);

              //
              mogoChassis.turnToHeading(350, 400);
              mogoChassis.moveToPoint(19, 21, 400, {.earlyExitRange = 10});
              intake.move(127);
               mogoChassis.moveToPoint(34, 48, 600, {.earlyExitRange = 10});
           mogoChassis.moveToPoint(43, 79, 1000, {.maxSpeed = 100, .earlyExitRange = 10});




          
           mogoChassis.moveToPoint(55,95 , 1000);
           
          //  mogoChassis.waitUntil(24);
          //  intake.move(100);
         mogoChassis.waitUntilDone();
          
          
        
         
          // moveFR(1690, 585, false, false, false, 80, 2000);
          twoRingChassis.moveToPoint(37, 53, 1500, {.forwards = false, .maxSpeed = 80, .earlyExitRange = 1});
          intake.move(0);
          setAutonState(1);
          twoRingChassis.waitUntil(22);
intake.move(127);
          twoRingChassis.turnToHeading(90, 500);
          
          // moveFR(1668, 535
          // , false, false, false, 80, 600);

          // moveR(544, false, false, 75,1300);
          //early into route so idk if we need distance


         
                    
          currentArmState = 1; 
                  loadActivated   = true; 
                  targetArmState  = LoadStates[1]; 
                  // moveDualFront(127, 127, true,true, true, 80, 1300);
                  twoRingChassis.moveToPoint(59, 53 , 1200, {.maxSpeed = 80});
                  twoRingChassis.waitUntilDone();
         setAutonState(2);

          
          

          // moveF(190, false, false, 80, 400);
          
          

          pros::delay(400);
          targetArmState = LoadStates[0];

            while (armSensor.get_position() > 12000 + 100) {
                  pros::delay(10);
                }
                
                pros::delay(100);
              
                intake.move(127);
                pros::delay(300); 
                intake.move(0);
                
                setAutonState(2);
                chassis.turnToHeading(90, 400);
                pros::delay(350);
                chassis.waitUntilDone();
                twoRingChassis.moveToPoint(45, twoRingChassis.getPose().y, 650, {.forwards = false});
          twoRingChassis.turnToHeading(180, 450);
          setAutonState(0);
         
         intake.move(127);
        

        
         twoRingChassis.moveToPoint(48, 20, 500, {.earlyExitRange = 10});
         // chassis.moveToPoint(-14, 30, 500, {.earlyExitRange = 5});
          twoRingChassis.turnToHeading(145, 400, {.earlyExitRange = 10});
          twoRingChassis.moveToPoint(58, 9, 800);

          // moveFL(268, 198, true, true ,true, 85, 600);

          // chassis.moveToPoint(-18, 46, 1000);

          twoRingChassis.moveToPoint(46, 26, 600, {.forwards = false});

          twoRingChassis.turnToHeading(178, 400);
          twoRingChassis.waitUntilDone();

        
          moveF(200, true, true, 75, 1500);
          pros::delay(100);
          
          
          
          twoRingChassis.turnToHeading(290, 700);
          twoRingChassis.waitUntilDone();
                twoRingChassis.moveToPoint(58, -7, 650, {.forwards = false});
         
          //drop first mogo
          mogo.set_value(false);
                
          



          ///////////////// Q2 start
          twoRingChassis.waitUntilDone();
          chassis.moveToPoint(-1, 0 ,1800);
          intake.move(0);
          chassis.turnToHeading(91, 550);
                chassis.moveToPoint(-21, 0, 2000, {.forwards = false, .maxSpeed = 50, .earlyExitRange = 3});
                chassis.waitUntil(17);
                mogo.set_value(true);
 

   mogoChassis.turnToHeading(-350, 500, {.earlyExitRange =10});
      mogoChassis.moveToPoint(-24, 18, 400, {.earlyExitRange = 10});
      intake.move(127);
      mogoChassis.moveToPoint(-38, 48, 600, {.earlyExitRange = 10});
      intake.move(127);
   mogoChassis.moveToPoint(-54, 74, 700, {.maxSpeed = 100, .earlyExitRange = 5});
intake.move(127);
mogoChassis.moveToPoint(-62, 87, 1100);
intake.move(127);
mogoChassis.waitUntilDone();

 //43, 77
   //36, 54


   /*

   */
intake.move(127);
   twoRingChassis.moveToPoint(-41, 41, 1500, {.forwards=  false, .earlyExitRange = 5});
   twoRingChassis.turnToHeading(200, 550, {.direction = lemlib::AngularDirection::CW_CLOCKWISE, .earlyExitRange = 5});

   twoRingChassis.moveToPoint(-45, 15 , 600, {.earlyExitRange = 6});

   twoRingChassis.turnToHeading(180, 250);
   twoRingChassis.waitUntilDone();
   
   
            // moveF(300, true, true, 65, 1000);
            // currentIntakeCommand = STOPSECOND;
            
            moveF(185, true, true, 60, 2000);
            
            twoRingChassis.turnToHeading(300, 500);
            twoRingChassis.moveToPoint(-57, 3, 1000);
            twoRingChassis.waitUntil(12);
            currentIntakeCommand = STOPFIRST;
          
            twoRingChassis.moveToPoint(-57, -13 , 1000, {.forwards = false});
            setAutonState(1);
            twoRingChassis.waitUntil(8);
            mogo.set_value(false);
           


           
            
            
            twoRingChassis.waitUntilDone();

            chassis.moveToPoint(-44, 39   , 1800, {.maxSpeed = 80});
            intake.move(127);
            chassis.waitUntilDone();
            chassis.turnToHeading(263, 700);
            
  currentArmState = 1; 
  loadActivated   = true; 
  targetArmState  = LoadStates[1]; 

chassis.moveToPoint(-65 , 40, 1200, {.maxSpeed = 80, .earlyExitRange = 1});    
chassis.waitUntilDone();

         
          setAutonState(2);
          

         
          pros::delay(450);
         setAutonState(1);

            while (armSensor.get_position() > 12000 + 100) {
                  pros::delay(10);
                }
                
                pros::delay(150);
              
                intake.move(127);
                pros::delay(300); 
                intake.move(0);
                
                setAutonState(2);
               
                pros::delay(350);
                setAutonState(1);
                chassis.moveToPoint(-48, 40, 1000, {.forwards = false});
                chassis.waitUntilDone();
                chassis.turnToHeading(350, 600);
                setAutonState(1);
                chassis.moveToPoint(-55,75, 700, {.earlyExitRange = 10});
                chassis.moveToPoint(-55,83, 1000, {.maxSpeed = 50});
                intake.move(127);
                chassis.moveToPoint(-40, 81, 700, {.forwards = false});

                
                chassis.turnToHeading(220, 400);
                chassis.moveToPoint(-31, 105, 1500, {.forwards = false, .maxSpeed = 55});
              
                chassis.waitUntil(20);
                mogo.set_value(true);
                pros::delay(100);
                 chassis.waitUntilDone();
                mogoChassis.turnToHeading(280, 400);
                currentArmState = 1; 
                                  loadActivated   = true; 
                                  targetArmState  = LoadStates[1]; 
                
               mogoChassis.moveToPoint(-66, 105, 1000, {.maxSpeed=85});
               doinker.set_value(true);
               mogoChassis.waitUntil(7);
               intake.move(0);
             
               pros::delay(200);
               setAutonState(1);
                mogoChassis.waitUntilDone();
                
mogoChassis.turnToHeading(190, 1000, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .minSpeed = 85});
pros::delay(500);
doinker.set_value(false);
mogoChassis.turnToHeading(135, 500, {.minSpeed = 70});
mogoChassis.waitUntilDone();
doinker.set_value(false);

// //           ///////////////QUADRANT 3.5 ?/////////////////////////
// //           mogoChassis.setPose(0,0, -123);
//           intake.move(127);
          mogoChassis.moveToPoint(-67, 104, 1000, {.forwards = false});
          mogoChassis.waitUntil(2);
          mogo.set_value(false);
          intake.move(-20);
          mogoChassis.waitUntilDone();
          currentArmState = 1; 
          loadActivated   = true; 
          targetArmState  = LoadStates[1];
          intake.move(127);
          //start of third mogo



          
          chassis.moveToPoint(-37, 94.5, 1000);
          
          pros::delay(310);
          
          intake.move(0);
          doinker2.set_value(true);
         
          chassis.turnToHeading(260, 600);
          intake.move(0);
          setAutonState(1);

         //third mogo grab
          chassis.moveToPoint(-12, 95, 1600, {.forwards = false, .maxSpeed = 55});
          
          
          chassis.waitUntil(18);
          intake.move(20);
          mogo.set_value(true);
          doinker2.set_value(false);
         
          
          


          chassis.waitUntilDone();
         intake.move(0);
          moveF(1660, true, true, 55 , 800);
           //alliance stake
          mogoChassis.turnToHeading(355, 600);
        
         //  mogoChassis.moveToPoint(-13, 107, 800, {.maxSpeed = 80, .earlyExitRange = 2});
          mogoChassis.waitUntilDone();
         
          moveF(310, false, false, 65, 700);
          setAutonState(3);
          pros::delay(450);
          pros::Task sortingTask(sortingControlTask); 
          intake.move(127);
          
          moveF(600, false, false, 65, 400);
          setAutonState(0);
          

        intake.move(127);
         mogoChassis.turnToHeading(245, 500);
         
          mogoChassis.moveToPoint(-40, 67, 1000);
          mogoChassis.turnToHeading(200, 500);
          mogoChassis.moveToPoint(1, 94, 1000, {.forwards = false});
          intake.move(127);
          mogoChassis.turnToHeading(143, 600);
          mogoChassis.moveToPoint(9, 79, 800);
          mogoChassis.turnToHeading(50, 550);
          mogoChassis.moveToPoint(30, 96, 600, {.earlyExitRange = 5});
          currentIntakeCommand = STOPBLUE;
          mogoChassis.turnToHeading(0, 200);
         
          mogoChassis.moveToPoint(30, 108, 1000);
                
          intake.move(127);
          
        
          mogoChassis.waitUntilDone();
          mogoChassis.moveToPoint(38, 107, 850, {.forwards =false, .minSpeed = 80}); // 35, 105
          doinker2.set_value(true);
          mogoChassis.turnToHeading(236, 1000, {.direction =  lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 75});
          
          mogoChassis.moveToPoint(42, 116, 1000, {.forwards = false});
          intake.move(127);
          mogoChassis.waitUntil(5);
          mogo.set_value(false);
          mogoChassis.swingToHeading(245, lemlib::DriveSide::RIGHT, 600, {.minSpeed = 127});
                mogoChassis.waitUntilDone();
                intake.move(-127);
     
          // mogoChassis.moveToPoint(36, 113, 500, {.minSpeed = 70});
          // mogoChassis.waitUntilDone();
          // doinker2.set_value(true);
      
          
          
          // mogoChassis.turnToHeading(225, 650, {.direction = lemlib::AngularDirection::CW_CLOCKWISE}) ;
          // mogoChassis.moveToPoint(44, 120, 700, {.forwards = false, .minSpeed= 50});

          // mogoChassis.waitUntil(6);
          // mogo.set_value(false);
          // intake.move(0);
          // mogoChassis.waitUntilDone();
          // mogoChassis.moveToPoint(42, 119, 500);
             
          
//           // pros::delay(150);
//           // mogo.set_value(true);
//           // pros::delay(150);

          
// //           left_motors.move(0);
// //           right_motors.move(0);
// //           mogo.set_value(true);
//           chassis.waitUntilDone();

//           left_motors.set_brake_mode(pros::MotorBrake::hold);
//           right_motors.set_brake_mode(pros::MotorBrake::hold);
//           mogoChassis.turnToHeading(295, 600);
//             pros::delay(250);
//             currentArmState = 1; 
//                   loadActivated   = true; 
//                   targetArmState  = LoadStates[1]; 



//                doinker.set_value(true);      
//    mogoChassis.waitUntilDone();
//    intake.move(100);
//    mogoChassis.moveToPoint(-4.5, 119, 1200,{.maxSpeed = 80});
//   mogoChassis.waitUntil(20);
//  intake.move(0);
// mogoChassis.turnToHeading(215, 800, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .minSpeed = 85});

// mogoChassis.turnToHeading(155, 600);
// intake.move(127);
// currentArmState = 1; 
//                   loadActivated   = true; 
//                   targetArmState  = LoadStates[1]; 
//   //  mogoChassis.turnToHeading(230, 450, {.maxSpeed = 85,.earlyExitRange = 10});
//   //   mogoChassis.turnToHeading(280, 400);
//       // moveF(500, true, true, 60, 1000);
//       doinker.set_value(false);
//       // mogoChassis.moveToPose(-3, 0, 139, 1500, {.maxSpeed = 70});

      
//   //  mogoChassis.turnToHeading(138, 1000);




//    mogoChassis.waitUntilDone();
//    chassis.waitUntilDone();
//     mogo.set_value(false);
//    pros::delay(20);
//    left_motors.move(-65);
//    right_motors.move(-65);
//    pros::delay(800);
// // left_motors.move(-25);
// //    right_motors.move(-25);
   
//       left_motors.move(0);
//    right_motors.move(0);



// //pre
// // setAutonState(1);
// // pros::delay(200);
// // intake.move(127);
// // pros::delay(250);
// //             pros::delay(250);
// //             currentArmState = 1; 
// //                   loadActivated   = true; 
// //                   targetArmState  = LoadStates[1]; 





// chassis.setPose(0,0,129);
// mogoChassis.setPose(0,0,129);
// twoRingChassis.setPose(0,0,129);

//    pros::delay(20);
//    left_motors.move(120);
//    right_motors.move(120); 
//    pros::delay(200);
//          left_motors.move(0);
//    right_motors.move(0);


// moveL( 630, true, false, 70, 1000);
// left_motors.move(30);
//    right_motors.move(30);
//    pros::delay(100); 
//        left_motors.move(0);
//    right_motors.move(0);
// chassis.turnToHeading(271, 600);
// chassis.waitUntilDone();

// chassis.setPose(0,0,0);
// mogoChassis.setPose(0,0,0);
// twoRingChassis.setPose(0,0,0);


// chassis.moveToPoint(1, -40, 1500, {.forwards = false, .maxSpeed = 60});
// chassis.waitUntil(38);
// mogo.set_value(true);
// chassis.waitUntilDone();
// moveF(1710, true, true, 50, 800);
// mogoChassis.turnToHeading(92, 600);
// mogoChassis.waitUntilDone();


// //alliance stake
// moveF(342, true, true, 70, 650);
// setAutonState(3);
// intake.move(127);
// pros::delay(400);
// mogoChassis.waitUntilDone();
// intake.move(127);
// moveF(560, false, false, 80, 400);
// setAutonState(0);
// mogoChassis.moveToPoint(-21, -62,1000, {.earlyExitRange = 2});
// intake.move(127);

// mogoChassis.turnToHeading(130, 500);
// intake.move(127);
// mogoChassis.waitUntilDone();
// // mogoChassis.moveToPoint(15, -74, 1000, {.maxSpeed = 90});

// moveF(400, true, true, 80, 1000);
// pros::delay(500);
// // intake.move(127);

// // intake.move(127);
// // mogoChassis.moveToPoint(15, -74, 1500, {.earlyExitRange = 1});
// // intake.move(127);
// // mogoChassis.moveToPoint(11, -80, 600, {.forwards = false, .minSpeed = 75});
// chassis.turnToHeading(215, 550);
// intake.move(127);
// chassis.waitUntilDone();
// doinker.set_value(true);
// mogoChassis.turnToHeading(-45, 1000, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE});
// left_motors.set_brake_mode(pros::MotorBrake::coast);
// right_motors.set_brake_mode(pros::MotorBrake::coast);

// mogoChassis.waitUntilDone();


// mogo.set_value(false);

// left_motors.move(-90);
// right_motors.move(-90);
// pros::delay(650);
// intake.move(-127);
// left_motors.move(40);
// right_motors.move(40);
// pros::delay(300);
// left_motors.move(0);
// right_motors.move(0);

// //    // // mogoChassis.swingToHeading(145, lemlib::DriveSide::LEFT, 1000, {.minSpeed = 40});


// // //     // mogoChassis.turnToHeading(15, 650, {.direction = lemlib::AngularDirection::CW_CLOCKWISE});
// // //     mogoChassis.waitUntilDone();
// // //     mogo.set_value(false);
// // //     pros::delay(100);
// // //     moveBR(435, 285, false, true, true, 55, 800);
    

// //     left_motors.move(40);
// //     right_motors.move(40);
// //     pros::delay(500);
// //      left_motors.move(-55);
// //     right_motors.move(-55);
// //     pros::delay(500);
// //        left_motors.move(0);
// //     right_motors.move(0);

    
    
    
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
