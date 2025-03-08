#include "main.h"
#include "bennyHeaders/hardwareAndSensors.h"
#include "bennyHeaders/macros.h"
#include "bennyHeaders/autons.h"
#include "bennyHeaders/DistanceSensorMovements.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "robotConfigs.h"
#include "sortingControl.h"





void initialize() {
  std::int32_t set_integration_time(20);
    pros::lcd::initialize(); 
    armSensor.reset_position();
    
    chassis.calibrate(); 
    
    hooks_rot.reset_position();

    pros::Task  preLoadControlTask([] {
      while(true) {
      preLoadControl();
      pros::delay(10);
      }
      
    });

    
    
    
    
    

    pros::Task autoWsTask([] {
        while (true) {
          AutopreLoadControl();
          pros::delay(10);
        }
      
    });


    // pros::Task autoLoadTask([] {
    //   while(true) {
    //     liftWsAuto();
    //   }
    // // });
    pros::Task secondScoreTask([] {
      while (true) {
        secondScoreControl();
        pros::delay(10);
      }
    });
    pros::Task LoadControlTask([] {
      while (true) {
        ArmStateControl();
        pros::delay(10);
      }

      
    });


// pros::Task mancoltask([&]()  {
//   while (true) {
//     manCol();
//   }
// });
// pros::Task pidInit([] {
// while (true) {
//   lemlib::Chassis chassis(drivetrain, latNoMogo, angNoMogo, sensors, &throttle_curve, &steer_curve);
// }
// });
pros::Task autoColorSortTask([] {
  while (true) {
  void sortingControlTask();
  pros::delay(10);
  }
});
sorter.set_led_pwm(35);


pros::Task screen_task([&]() {
        while (true) {
            pros::lcd::print(0, "X: %f", chassis.getPose().x);
            pros::lcd::print(1, "Y: %f", chassis.getPose().y);
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta);  
            pros::delay(20);
        }
    });
}



void disabled() {
mogo.set_value(false);


  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
}


void competition_initialize() {
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
}


void autonomous() {
chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);


     int autonSelector = 1;  

    switch (autonSelector) {
        case 0:
    
        bristolSkills();
        break;

        case 1:
        rightSideMidRush();
            break;
        case 2:
      leftSideMidRush();
            break;
        case 3:
         
            blueSideMogoRush();
            break;
        case 4:
  
            break;
        case 5:
            blueSawp();
            break;
        case 6:
        redSawp();
            break;
            case 7:

            redSawp2();
            break;
        case 8:

            red6SAWP();
            break;

    }
    


    
}












bool mogoActivated = false;
bool intakeActivated = false;
bool doinkerActivated = false;
bool doinker2Activated = false;






void opcontrol() {
// rightSideMidRush();
  // bristolSkills();
  // pros::Task sortingTask(sortingControlTask); 
// autonDisabled = true;


chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
// setAutonState(1);
//               pros::delay(320);
//                   intake.move(127);
//               pros::delay(300);
//               intake.move(0);
//               pros::delay(40);


//     setAutonState(3);
//     pros::delay(500);
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
  pros::Controller master(pros::E_CONTROLLER_MASTER);
  
  
  while (true) {
  
    // if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
    //   mogoChassis.moveToPoint(0, 24, 1500);
    //   mogoChassis.moveToPoint(0, 0,1500, {.forwards = false});
    // }
    std::cout << "Motor Temperature: " << left_motors.get_temperature_all()[1];
    int leftY  = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
    chassis.arcade(leftY, rightX);

      
    if (!intakeOverride) {
      if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
        intake.move(127);
      }
      else {
        intake.move(0);
      }
    }

    
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
      cycleLoadState();
    }

    
    
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
      
      if (currentArmState == 0) {
        
        currentArmState = 1; 
        loadActivated   = true; 
        targetArmState  = LoadStates[1]; 
      } 
      else {
        
        cycleTipState();
      }
    }

    
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
      cycleDescoreState();
    }

    
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
      doinker2Activated = !doinker2Activated;
      doinker2.set_value(doinker2Activated);
    pros::delay(60);
    }

    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
         resetArmPos();
    }
    
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
    doinkerActivated = !doinkerActivated;
   doinker.set_value(doinkerActivated);
   pros::delay(20);
// pros::Task sort(autoColorSort);
    // cycleTipState();
    }

    
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
      mogoActivated = !mogoActivated;
      mogo.set_value(mogoActivated);
    }

    pros::delay(20);
  }


  if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
    // sortingTaskRunning = !sortingTaskRunning;
    // allianceStake();
    // intakeLift.set_value(true);
  }
}








