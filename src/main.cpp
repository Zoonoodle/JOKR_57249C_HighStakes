#include "main.h"
#include "bennyHeaders/hardwareAndSensors.h"
#include "bennyHeaders/macros.h"
#include "bennyHeaders/autons.h"
#include "bennyHeaders/DistanceSensorMovements.hpp"
#include "bennyHeaders/MCLSystem.hpp"
#include "bennyHeaders/RouteRecorder.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "robotConfigs.h"
#include "sortingControl.h"

// Forward declaration of task for MCL touch handling
void mclTouchHandlerTask(void*);

// Forward declaration for route recorder menu
void showRouteMenu();

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

    // Initialize MCL system with a typical starting position
    // Replace these values with your actual starting position
    initializeMCL(0, 0, 90);
    
    // Create task to handle touch events for MCL visualization
    pros::Task mclTouchTask(mclTouchHandlerTask);

    pros::Task autoWsTask([] {
        while (true) {
          AutopreLoadControl();
          pros::delay(10);
        }
      
    });

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
  // Set up MCL for autonomous with the actual starting position
  // These values should match your actual starting position on the field
  setupMCLForAutonomous(0, 0, 90);
  
  // Display the MCL visualization
  displayMCL();
  
  // Run your autonomous routines...
  // You can now use getMCLPose() or getFusedPosition() to get position estimates
  // during autonomous
  
  // Example of how to use the fused position:
  // lemlib::Pose currentPos = getFusedPosition();
  // chassis.moveTo(currentPos.x + 500, currentPos.y, 1000);

  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);


     int autonSelector = 0;  

    switch (autonSelector) {
        case 0:
    
        statesSkills();
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

// Show route recording menu on the LCD screen
void showRouteMenu() {
  pros::lcd::clear();
  pros::lcd::print(0, "==== Route Recorder ====");
  
  if (recorder.isRecording()) {
    pros::lcd::print(1, "Recording... %d frames", recorder.getFrameCount());
    pros::lcd::print(2, "UP: Stop and Save");
    pros::lcd::print(3, "DOWN: Stop and Discard");
  } else {
    std::vector<std::string> routes = recorder.listRecordings();
    
    pros::lcd::print(1, "UP: Start Recording");
    pros::lcd::print(2, "DOWN: List Routes (%d)", routes.size());
    
    if (!routes.empty()) {
      pros::lcd::print(3, "Latest: %s", routes.back().c_str());
    }
  }
}

void opcontrol() {
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
  
  // Initialize display and buttons for MCL control in driver control
  pros::lcd::set_text(0, "MCL System Controls:");
  pros::lcd::set_text(1, "A - Reset MCL    B - Display MCL");
  pros::lcd::set_text(2, "X - Correct Odom Y - Simulate Path");
  
  pros::Controller master(pros::E_CONTROLLER_MASTER);
  
  // Show the route recorder menu initially
  showRouteMenu();
  
  while (true) {
    std::cout << "Motor Temperature: " << left_motors.get_temperature_all()[1];
    int leftY  = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
    chassis.arcade(leftY, rightX);
    
    // MCL control with controller buttons
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
      // Reset MCL with current odometry position
      resetMCL();
      master.rumble(".");
    }
    
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
      // Display MCL visualization
      displayMCL();
    }
    
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
      // Correct odometry with MCL estimate
      correctOdometryWithMCL();
      master.rumble("..");
    }
    
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
      // Simulate a path to a point 1 meter in front of current position
      lemlib::Pose current = chassis.getPose();
      double theta = current.theta * (M_PI / 180.0);
      double targetX = current.x + 1000 * cos(theta); // 1 meter forward
      double targetY = current.y + 1000 * sin(theta);
      simulatePathToTarget(targetX, targetY, current.theta);
    }
    
    // Route recorder controls
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
      if (!recorder.isRecording()) {
        // Start a new recording
        recorder.startRecording();
        master.rumble("-");
        showRouteMenu();
      } else {
        // Stop recording and save
        recorder.stopRecording();
        
        // Generate filename with date/time
        char filename[32];
        std::time_t now = std::time(nullptr);
        std::tm* tm = std::localtime(&now);
        std::strftime(filename, sizeof(filename), "route_%Y%m%d_%H%M%S", tm);
        
        if (recorder.saveToSD(filename)) {
          master.rumble("--");
        } else {
          master.rumble("---");
        }
        
        showRouteMenu();
      }
    }
    
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
      if (recorder.isRecording()) {
        // Stop recording without saving
        recorder.stopRecording();
        master.rumble("-.");
        showRouteMenu();
      } else {
        // Show list of recordings and allow playback
        std::vector<std::string> routes = recorder.listRecordings();
        
        if (routes.empty()) {
          pros::lcd::clear();
          pros::lcd::print(0, "No recordings found");
          pros::lcd::print(1, "Press any button to return");
          
          // Wait for button press
          while (!master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A) &&
                 !master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B) &&
                 !master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X) &&
                 !master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y) &&
                 !master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP) &&
                 !master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN) &&
                 !master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT) &&
                 !master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            pros::delay(20);
          }
        } else {
          // Show the latest recording and allow playback
          std::string latestRoute = routes.back();
          
          pros::lcd::clear();
          pros::lcd::print(0, "Play back recording?");
          pros::lcd::print(1, "%s", latestRoute.c_str());
          pros::lcd::print(2, "A: Play with MCL  B: Cancel");
          
          // Wait for button press
          while (true) {
            if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
              // Play with MCL visualization
              recorder.playbackFromSD(latestRoute, true);
              break;
            } else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
              break;
            }
            pros::delay(20);
          }
        }
        
        // Return to main menu
        showRouteMenu();
      }
    }
    
    // Regular drive controls
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

// MCL touch handler task
void mclTouchHandlerTask(void*) {
  while (true) {
    handleMCLTouch();
    pros::delay(50); // Check for touches every 50ms
  }
}








