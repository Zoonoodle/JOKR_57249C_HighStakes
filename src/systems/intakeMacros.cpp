#include "bennyHeaders/autons.h"
#include "pros/abstract_motor.hpp"
#include "pros/rtos.hpp"
#include "robotConfigs.h"
#include "sortingControl.h"
#include "bennyHeaders/hardwareAndSensors.h"


intakeCommand currentIntakeCommand = INTAKENONE;
sortingCommand currentSortingCommand = SORTINGNONE;



bool intakeTaskRunning = true;
bool sortingTaskRunning = true;
bool isRed = true;
bool ejecting = false;

void intakeControlTask(void *param) {
   while (intakeTaskRunning) {
        switch (currentIntakeCommand) {

         case INTAKENONE: 
         pros::delay(10); //hey
         break;

          case STOPSECOND:
while (sorter.get_proximity() < 100 ) {
                    pros::delay(10);
                }
                  hooks_rot.reset_position();
                while(hooks_rot.get_position() < 53000) {
                    pros::delay(10);
                }
                while (sorter.get_proximity() < 100) {
                    pros::delay(10);
                }
                
                
                intake.move(0);
                // pros::delay(20);
                // intake.move(-40);
                
                break;
                

             
                case STOPFIRST:
                while (sorter.get_proximity() < 100) {
                    pros::delay(10);
                }
               intake.move(0);
                
                currentIntakeCommand = INTAKENONE;
                break;
            case STOPAFTER:
                while (sorter.get_proximity() < 100 ) {
                    pros::delay(10);
                }
                  hooks_rot.reset_position();
                while(hooks_rot.get_position() < 53000) {
                    pros::delay(10);
                }
                while (sorter.get_proximity() < 100) {
                    pros::delay(10);
                }
                 hooks_rot.reset_position();
                while(hooks_rot.get_position() < 53000) {
                    pros::delay(10);
                }
                pros::delay(400);
               intake.move(0);
                
            currentIntakeCommand = INTAKENONE;
            break;
            case STOPTHIRD:
            while (sorter.get_proximity() < 100 ) {
                    pros::delay(10);
                }
                  hooks_rot.reset_position();
                while(hooks_rot.get_position() < 83000) {
                    pros::delay(10);
                }
                while (sorter.get_proximity() < 100) {
                    pros::delay(10);
                }
                  hooks_rot.reset_position();
                while(hooks_rot.get_position() < 56000) {
                    pros::delay(10);
                }
                while (sorter.get_proximity() < 100) {
                    pros::delay(10);
                }
                
                intake.move(0);
                currentIntakeCommand = INTAKENONE;
                break;
                case SORTFIRST:
                  while (sorter.get_proximity() < 150) {
                    pros::delay(10);
                  }
        
        hooks_rot.reset_position();
       

        // if ((sorter.get_hue() > 350) || (sorter.get_hue() < 10)) {

            
       
            intake.set_brake_mode(pros::MotorBrake::brake);
            
            while(hooks_rot.get_position() < 39000) {
                    intake.move(100);
                }


            pros::delay(100);
            intake.move(-127);
            pros::delay(200);
            intake.move(0);
            pros::delay(100);
            intake.move(127);
                currentIntakeCommand = INTAKENONE;
                break;
                case STOPRED:
                while (sorter.get_proximity() < 150) {
                 pros::delay(10);
                }
                if ((sorter.get_hue() > 350) || (sorter.get_hue() < 10)) {
                    intake.move(0);
                }
currentIntakeCommand = INTAKENONE;
                break;
                
                case STOPBLUE:
                while (sorter.get_proximity() < 150) {
                 pros::delay(10);
                }
                if ((sorter.get_hue() > 180) && (sorter.get_hue() < 300)) {
                    intake.move(0);
                }
currentIntakeCommand = INTAKENONE;
                break;



                case unclampAfterOne:
                while (sorter.get_proximity() < 100 ) {
                    pros::delay(10);
                }
                  hooks_rot.reset_position();
                while(hooks_rot.get_position() < 90000) {
                    pros::delay(10);
                }
                
                pros::delay(100);


                mogo.set_value( false);
                currentIntakeCommand = INTAKENONE;
                break;
        }
   }
}




void sortingControlTask(void *param) {
    while (sortingTaskRunning) {
    switch (currentSortingCommand) {

        case SORTINGNONE:
        while (sorter.get_proximity() < 150) {
            pros::delay(10);
                }

                if (isRed) {
                    currentSortingCommand = SORT_BLUE;
                }
                else {
                    currentSortingCommand = SORT_RED;
                }

            break;


            case SORT_RED:
    
        ejecting = true;
        hooks_rot.reset_position();
        
         

        if ((sorter.get_hue() > 350) || (sorter.get_hue() < 10)) {

            
           
            intake.set_brake_mode(pros::MotorBrake::brake);
            
            while (sorter.get_proximity() > 150 ) {
                pros::delay(10);
            }
            intakeOverride = true;
            while(hooks_rot.get_position() < 7200) {
                intake.move(127);
            }
            


         
            
            intake.move(-127);
            pros::delay(120);
            intake.move(127);
            intakeOverride=false;
            ejecting = false;

        }
        currentSortingCommand = SORTINGNONE;
        break;

case SORT_BLUE:
ejecting = true;
        hooks_rot.reset_position();
        
         

        if ((sorter.get_hue() > 180) && (sorter.get_hue() < 300)) {

            
            
            intake.set_brake_mode(pros::MotorBrake::brake);
            

            while (sorter.get_proximity() > 150 ) {
                pros::delay(10);
            }
            intakeOverride = true;
            while(hooks_rot.get_position() < 7200) {
                intake.move(127);
            }


         
            intake.move(-127);
            pros::delay(120);
            intake.move(127);
            intakeOverride=false;
            ejecting = false;

        }
        currentSortingCommand = SORTINGNONE;
break;
    }
    
      pros::delay(10);
    }


   





// void manCol() {
    
   
//    if(!sortOn) return;
//    intake.move(127);
//     while (sorter.get_proximity() < 150 && sortOn ) 
//         pros::delay(10);
//       hooks_rot.reset_position();
//    while (hooks_rot.get_position() < 46000 && sortOn) {
//                 pros::delay(10);
//             }
 
   
//     intake.move(0);
//     pros::delay(200);
   


   
//     sortOn = false;
//     pros::delay(10);
    


   
// }
}