#include "robotConfigs.h"
#include "sortingControl.h"
#include "bennyHeaders/hardwareAndSensors.h"

sortingCommand currentSortingCommand = SORTINGNONE;
bool sortingTaskRunning = true;


void sortingControlTask(void *param) {
   while (sortingTaskRunning) {
        switch (currentSortingCommand) {

         case SORTINGNONE: 
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
                pros::delay(20);
                intake.move(-100);
                pros::delay(100);
                intake.move(0);
                currentSortingCommand = SORTINGNONE;
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
                
            currentSortingCommand = SORTINGNONE;
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
                currentSortingCommand = SORTINGNONE;
                break;
                
        }
   }
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
