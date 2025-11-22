#include "main.h"
#include "pros/distance.hpp"
#include "pros/misc.h"
#include "pros/motors.hpp"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/optical.hpp"
#include "pros/rtos.hpp"
#include "pros/screen.hpp"
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <string>
using namespace std;
using namespace pros;
pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::Optical colorSense(9);
pros::MotorGroup leftMotors({11 , -15, -13}, pros::MotorGearset::blue); // left motors on ports 1, 2, 3
pros::MotorGroup rightMotors({-20, 19,18}, pros::MotorGearset::blue); // right motors on ports 4, 5, 6
//first = front
// other motors
pros::Motor firstStage(-14, pros::MotorGearset::blue);
pros::Motor upperIntake(2, pros::MotorGearset::blue);
bool load = false;
bool unload = false;

// distance sensors
pros::Distance frontDS(1);
pros::Distance rightDS(5);

// pnaumatics
pros::adi::DigitalOut tongue('B');
pros::adi::DigitalOut wings('A');
pros::adi::DigitalOut hood('C');
pros::adi::DigitalOut midGoal('D');
pros::adi::DigitalOut doublep('H');

bool stopSkills = false;
// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              11, // 10 inch track width
                              lemlib::Omniwheel::NEW_275,
                              450, // drivetrain rpm is 450
                              0 // horizontal drift is 2 (for now)
);
// create an imu on port 3
pros::Imu imu1(3);
// create a v5 rotation sensor on port 1
pros::Rotation vertical_encoder(-8);
// horizontal tracking wheel
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_275,0);
lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            nullptr, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu1 // inertial sensor
);

// lateral PID controller //7 and 30 original; 8/15; 4-5 wirh low kd work; 4=kp and 0.005 works for ki??? adjust kd accordingly
 lemlib::ControllerSettings lateral_controller(4, // proportional gain (kP)
                                              .005, // integral gain (kI)
                                              0, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              10 // maximum acceleration (slew)
);
//7,30
// angular PID controller
//tried 6.6 and greater and none worked 
//6.5 40 --to within +-0.2 degrees @90 degree clocwise turn
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              12, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// create the chassis
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);
void antiJam() {
    
    hood.set_value(true);
    while (true) {
        if (std::abs(firstStage.get_actual_velocity()) < 20) {
            firstStage.move(-127);
            upperIntake.move(127);
        }
        else {
            firstStage.move(127);
            upperIntake.move(-127);
        }
        pros::delay(50);
    }
}
/*
void intake() {
    while (load) {
    firstStage.move(127);
    if (colorSense.get_proximity() < 200) {
        upperIntake.move(-127);
    }
    else {
        upperIntake.move(0);
    }
    pros::delay(50);
}

}*/
time_t lastChange = time_t();
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    //intake();
    //antiJam();
    // print position to brain screen
    pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            //pros::lcd::print(0, "Dist: %i", colorSense.get_proximity());
            pros::lcd::print(0, "vel: %f", firstStage.get_actual_velocity()); // x
            pros::lcd::print(1, "vel1: %f", upperIntake.get_actual_velocity()); // x
            /*pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y*/
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            //string toPrint = "(" + to_string((int)chassis.getPose().x) + "," + to_string((int)chassis.getPose().y) + "," + to_string((int)chassis.getPose().theta) + ")";
            string toPrint = to_string(frontDS.get()) + " " + to_string(rightDS.get()) + "          ";
            controller.set_text(0,0,toPrint);
            // delay to save resources
            pros::delay(50);
            if (stopSkills) {
                leftMotors.brake();
                rightMotors.brake();
            }
            else {
            if (load) {
                firstStage.move(127);
                if (colorSense.get_proximity() < 200) {
                upperIntake.move(-127);
                }
                else {
                upperIntake.move(0);
                }
                pros::delay(10);
                if (std::abs(firstStage.get_actual_velocity()) < 20) {
            firstStage.move(-127);
            upperIntake.move(127);
        }

            }
            if (unload) {
                hood.set_value(true);
                firstStage.move(127);
                upperIntake.move(-127);
                pros::delay(10);
                if (std::abs(firstStage.get_actual_velocity()) < 20 || std::abs(upperIntake.get_actual_velocity()) < 20) {
            firstStage.move(-127);
            upperIntake.move(127);
        }
 
            } 
        }
        }
    });
}

/* 
    0 - lateral pid test
    1 - SAWP (middle, right side)
    2 - solo win point (right/low goal)
    3 - solo win point (left/middle goal)
    4 - left match
    5 - skills (aneeks field)
    6 - right match
    7 - intake test
    8 - elim left
    9 - ???
    10 - New Skills (half field 61 route)
*/
int chosenAuton = 2;
//

void autonomous() {
    switch(chosenAuton){
		// lateral pid test
        case 0:
        printf("test print");
        /*pros::delay(100);
            chassis.moveToPoint(0, 30, 2000);
            chassis.waitUntilDone();
            chassis.moveToPoint(0, 0, 2000, {.forwards = false});

            // end
            chassis.waitUntilDone();*/
            hood.set_value(true);
            break;
        // angular pid test
        case 1:
           // descore from tube and score on long goal
            chassis.setPose(0,0,0);
            chassis.moveToPoint(0, 38, 1000, {.maxSpeed = 100});
            chassis.waitUntilDone();
            tongue.set_value(true);
            chassis.turnToHeading(90, 500, {.maxSpeed = 127});
            chassis.waitUntilDone();
            load = true;
            chassis.moveToPoint(14.5, 38, 900,{.maxSpeed = 70});
            chassis.waitUntilDone();
            chassis.moveToPoint(-30, 43, 1000,{.forwards = false});
            chassis.waitUntilDone();
            load = false;
            unload = true;
            chassis.turnToHeading(90, 1000);
            chassis.waitUntilDone();
            chassis.setPose(0,0,0);
            unload = false;
            load = true;
            // get balls and score on low goal
            chassis.moveToPoint(0, 20, 750);
            chassis.waitUntilDone();
            tongue.set_value(false);
            chassis.turnToHeading(135, 500);
            chassis.waitUntilDone();
            chassis.moveToPoint(17, 0, 1000);
            chassis.moveToPoint(23, -5, 500, {.maxSpeed = 127});
            chassis.waitUntilDone();
            chassis.moveToPoint(36, -16, 500);
            chassis.waitUntilDone();
            load = false;
            chassis.turnToHeading(140, 750);
            firstStage.move(-127);
            chassis.waitUntilDone();
            // go to get other three balls
            chassis.moveToPoint(27, -8, 500, {.forwards = false});
            load=true;
            chassis.waitUntilDone();
            chassis.turnToHeading(90, 500);
            chassis.waitUntilDone();
            chassis.moveToPoint(65, -8, 750);
            chassis.waitUntilDone();
            tongue.set_value(true);
            // descore other side and score on other long goal
            chassis.moveToPoint(90, 20, 750);
            chassis.waitUntilDone();
            chassis.turnToHeading(0, 500);
            chassis.waitUntilDone();
            chassis.moveToPoint(89, 34.5, 750, {.maxSpeed = 70});
            chassis.turnToHeading(0, 500);
            chassis.waitUntilDone();
            chassis.moveToPoint(93, -15, 750, {.forwards = false});
            chassis.waitUntilDone();
            load=false; 
            unload=true;
            chassis.moveToPoint(93, -15, 2000, {.forwards = false});
            break;
        // simple match auto - aneek makes
        case 2:
            // descore from tube and score on long goal
            chassis.setPose(0,0,0);
            chassis.moveToPoint(0, 38, 1000, {.maxSpeed = 100});
            chassis.waitUntilDone();
            tongue.set_value(true);
            chassis.turnToHeading(90, 500, {.maxSpeed = 127});
            chassis.waitUntilDone();
            load = true;
            chassis.moveToPoint(14.5, 38, 900,{.maxSpeed = 70});
            chassis.waitUntilDone();
            chassis.moveToPoint(-30, 43, 1000,{.forwards = false});
            chassis.waitUntilDone();
            load = false;
            unload = true;
            chassis.turnToHeading(90, 1000);
            chassis.waitUntilDone();
            chassis.setPose(0,0,0);
            unload = false;
            load = true;
            // get balls and score on low goal
            chassis.moveToPoint(0, 20, 750);
            chassis.waitUntilDone();
            tongue.set_value(false);
            chassis.turnToHeading(135, 500);
            chassis.waitUntilDone();
            chassis.moveToPoint(17, 0, 1000);
            chassis.moveToPoint(23, -5, 500, {.maxSpeed = 127});
            chassis.waitUntilDone();
            chassis.moveToPoint(36, -16, 500);
            chassis.waitUntilDone();
            load = false;
            chassis.turnToHeading(140, 750);
            firstStage.move(-127);
            chassis.waitUntilDone();
            // go to get other three balls
            chassis.moveToPoint(27, -8, 500, {.forwards = false});
            load=true;
            chassis.waitUntilDone();
            chassis.turnToHeading(90, 500);
            chassis.waitUntilDone();
            chassis.moveToPoint(65, -8, 750);
            chassis.waitUntilDone();
            tongue.set_value(true);
            // descore other side and score on other long goal
            chassis.moveToPoint(90, 20, 750);
            chassis.waitUntilDone();
            chassis.turnToHeading(0, 500);
            chassis.waitUntilDone();
            chassis.moveToPoint(89, 34.5, 750, {.maxSpeed = 70});
            chassis.turnToHeading(0, 500);
            chassis.waitUntilDone();
            chassis.moveToPoint(93, -15, 750, {.forwards = false});
            chassis.waitUntilDone();
            load=false; 
            unload=true;
            chassis.moveToPoint(93, -15, 2000, {.forwards = false});
            break;
        case 3:
            // descore from tube and score on long goal
            chassis.setPose(0,0,0);
            chassis.moveToPoint(0, 41, 1000, {.maxSpeed = 100});
            chassis.waitUntilDone();
            tongue.set_value(true);
            chassis.turnToHeading(-90, 500, {.maxSpeed = 127});
            chassis.waitUntilDone();
            load = true;
            chassis.moveToPoint(-14.5, 39, 900,{.maxSpeed = 70});
            chassis.waitUntilDone();
            chassis.moveToPoint(30, 41, 1000,{.forwards = false});
            chassis.waitUntilDone();
            load = false;
            unload = true;
            chassis.turnToHeading(-90, 1000);
            chassis.waitUntilDone();
            chassis.setPose(0,0,0);
            unload = false;
            load = true;
            // get balls and score on mid goal
            chassis.moveToPoint(0, 20, 750);
            chassis.waitUntilDone();
            tongue.set_value(false);
            chassis.turnToHeading(-135, 500);
            chassis.waitUntilDone();
            chassis.moveToPoint(-25, -7, 1500, {.maxSpeed = 80});
            chassis.waitUntilDone();
            chassis.turnToHeading(45, 500);
            chassis.moveToPoint(-42, -20, 750, {.forwards = false});
            chassis.waitUntilDone();
            load = false;
            chassis.turnToHeading(40, 500);
            chassis.waitUntilDone();
            chassis.waitUntilDone();
            firstStage.move(127);
            upperIntake.move(90);
            midGoal.set_value(true);
            pros::delay(1000);
            midGoal.set_value(false);
            // go to get other three balls
            chassis.moveToPoint(-27, -5, 500);
            load=true;
            chassis.waitUntilDone();
            chassis.turnToHeading(-90, 500);
            chassis.waitUntilDone();
            chassis.moveToPoint(-70, -10, 750);
            chassis.waitUntilDone();
            tongue.set_value(true);
            // descore other side and score on other long goal
            chassis.moveToPoint(-98, 10, 1000);
            chassis.waitUntilDone();
            chassis.turnToHeading(0, 500);
            chassis.waitUntilDone();
            chassis.moveToPoint(-97, 27, 750, {.maxSpeed = 70});
            chassis.turnToHeading(0, 500);
            chassis.waitUntilDone();
            chassis.moveToPoint(-98, -15, 750, {.forwards = false});
            chassis.waitUntilDone();
            load=false; 
            unload=true;
            chassis.moveToPoint(-98, -15, 2000, {.forwards = false});

            break;
        case 4:
            chassis.setPose(0,0,0);
            chassis.moveToPoint(0, 38, 1000, {.maxSpeed = 127});
            chassis.turnToHeading(-90, 500, {.maxSpeed = 127});
            tongue.set_value(true);
            pros::delay(1250);
            firstStage.move(127);
            upperIntake.move(-127);
            chassis.moveToPoint(-12, 40, 1000,{.maxSpeed = 80});
            chassis.turnToHeading(-90, 500);
            chassis.moveToPoint(30, 42, 1000,{.forwards = false, .maxSpeed = 100});
            chassis.waitUntilDone();
            hood.set_value(true);
            // upperIntake.move(-127);
            pros::delay(2000);
            upperIntake.move(127);
            firstStage.move(-127);
            chassis.waitUntilDone();
            tongue.set_value(false);
            chassis.moveToPoint(0, 40, 1000,{.maxSpeed = 100});/*
            chassis.moveToPoint(30, 40, 1000,{.forwards = false, .maxSpeed = 90});*/
            chassis.turnToHeading(133, 750,{.maxSpeed = 100});
            chassis.waitUntilDone();
            upperIntake.move(-127);
            firstStage.move(127);
            chassis.moveToPoint(28, 13, 1000,{.maxSpeed = 100});
            chassis.waitUntilDone();
            tongue.set_value(true);
            chassis.turnToHeading(-47, 800,{.maxSpeed = 100});
            chassis.moveToPoint(36,5, 1000, {.forwards = false, .maxSpeed = 100});
            chassis.turnToHeading(-56, 200);
            chassis.waitUntilDone();
            upperIntake.move(-70);
            break;

        case 5:
            //prepare to collect from holder
            chassis.setPose(0,0,0);
            chassis.moveToPoint(0, 37, 1000, {.maxSpeed = 100});
            chassis.waitUntilDone();
            delay(10);
            tongue.set_value(true);
            chassis.turnToHeading(90, 500, {.maxSpeed = 127});
            chassis.waitUntilDone();
            
            // firstStage.move(127);
            // upperIntake.move(-127);
            load = true;
            chassis.moveToPoint(14, 37, 2000,{.maxSpeed = 70});
            chassis.waitUntilDone();
            /*
            chassis.moveToPoint(-30, 38.5, 1000,{.forwards = false, .maxSpeed = 100});
            chassis.waitUntilDone();
            // hood.set_value(true);
            load = false;
            unload = true;
            pros::delay(2000);
            chassis.waitUntilDone();
            tongue.set_value(false);
            // hood.set_value(false);
            chassis.turnToHeading(90, 500);
            chassis.waitUntilDone();
            chassis.setPose(0,0,imu1.get_heading());
            // after scoring get 4 blocks in 1st quad
            chassis.moveToPoint(20, 0, 1000);
            load = true;
            unload = false;
            chassis.waitUntilDone();
            chassis.turnToHeading(225, 500);
            chassis.waitUntilDone();
            chassis.moveToPoint(0, -15, 500);
            chassis.moveToPoint(-5, -23, 1000, {.maxSpeed = 50});
            pros::delay(250);
            tongue.set_value(true);
            chassis.waitUntilDone();
            pros::delay(100);
            chassis.turnToHeading(60, 500);
            chassis.moveToPoint(15, 0, 1000);
            chassis.waitUntilDone();
            chassis.turnToHeading(90, 500);
            chassis.waitUntilDone();
            chassis.moveToPoint(-10, 0, 1000, {.forwards = false});
            chassis.waitUntilDone();
            // hood.set_value(true);
            load = false;
            unload = true;
            pros::delay(1250);
            chassis.turnToHeading(90, 500);
            chassis.waitUntilDone();
            chassis.setPose(0,0,imu1.get_heading());
            hood.set_value(false);
            // move to far side of field
            chassis.moveToPoint(10, 0, 500);
            chassis.moveToPoint(15, -14, 750);
            unload = false;
            load = true;
            chassis.waitUntilDone();
            chassis.turnToHeading(270, 500);
            chassis.waitUntilDone();
            chassis.moveToPoint(-55,-9, 1500);
            chassis.moveToPoint(-75, 5, 1000);
            chassis.waitUntilDone();
            chassis.turnToHeading(270, 500);
            chassis.waitUntilDone();
            chassis.moveToPoint(-87, 4, 1750, {.maxSpeed = 70});
            chassis.waitUntilDone();
            chassis.turnToHeading(270, 500);
            chassis.waitUntilDone();
            chassis.moveToPoint(-40, 5, 1000, {.forwards = false});
            chassis.waitUntilDone();
            // hood.set_value(true);
            load = false;
            unload = true;
            pros::delay(1250);
            // hood.set_value(false);
            chassis.turnToHeading(270, 500);
            chassis.waitUntilDone();
            chassis.setPose(0,0,imu1.get_heading());
            // move to left side of field
            chassis.moveToPoint(-13, 0, 1000);
            chassis.waitUntilDone();
            unload = false;
            load = true;
            chassis.turnToHeading(180, 500);
            chassis.waitUntilDone();
            chassis.moveToPoint(-15, -86, 2300, {.maxSpeed = 80});
            chassis.waitUntilDone();
            chassis.turnToHeading(270, 500);
            chassis.waitUntilDone();
            chassis.moveToPoint(-35, -88, 2000, {.maxSpeed = 70});
            chassis.waitUntilDone();
            chassis.moveToPoint(10, -88, 1000, {.forwards = false});
            chassis.waitUntilDone();
            // hood.set_value(true);
            load = false;
            unload = true;
            pros::delay(1250);
            // hood.set_value(false);
            chassis.turnToHeading(270, 500);
            chassis.waitUntilDone();
            chassis.setPose(0,0,imu1.get_heading());
            // move to near left side of field
            chassis.moveToPoint(-10, 0, 500);
            chassis.moveToPoint(-15, 13, 750);
            unload = false;
            load = true;
            chassis.waitUntilDone();
            chassis.turnToHeading(90, 500);
            chassis.waitUntilDone();
            chassis.moveToPoint(55, 8, 1500);
            chassis.moveToPoint(75, -4, 1000);
            chassis.waitUntilDone();
            chassis.turnToHeading(90, 500);
            chassis.waitUntilDone();
            chassis.moveToPoint(87, -5, 1500, {.maxSpeed = 70});
            chassis.turnToHeading(90, 500);
            chassis.waitUntilDone();
            chassis.moveToPoint(40, -4, 1000, {.forwards = false});
            chassis.waitUntilDone();
            // hood.set_value(true);
            load = false;
            unload = true;
            pros::delay(1250);
            // hood.set_value(false);
            tongue.set_value(false);
            chassis.turnToHeading(90, 500);
            chassis.waitUntilDone();
            chassis.setPose(0,0,imu1.get_heading());
            // get other 4 blocks and score on long goal
            chassis.moveToPoint(20, 0, 1000);
            chassis.waitUntilDone();
            unload = false;
            load = true;
            chassis.turnToHeading(315, 500);
            chassis.waitUntilDone();
            chassis.moveToPoint(0, 17, 500);
            chassis.moveToPoint(-2, 26, 1000, {.maxSpeed = 50});
            pros::delay(300);
            tongue.set_value(true);
            chassis.waitUntilDone();
            pros::delay(100);
            chassis.turnToHeading(120, 500);
            chassis.moveToPoint(15,2, 1000);
            chassis.waitUntilDone();
            chassis.turnToHeading(90, 500);
            chassis.waitUntilDone();
            chassis.moveToPoint(-10, 2, 1000, {.forwards = false});
            chassis.waitUntilDone();
            // hood.set_value(true);
            load = false;
            unload = true;
            pros::delay(1250);
            chassis.turnToHeading(90, 500);
            chassis.waitUntilDone();
            chassis.setPose(0,0,imu1.get_heading());
            unload = false;
            // clear and park
            chassis.moveToPoint(25, 0, 750);
            chassis.waitUntilDone();
            chassis.turnToHeading(200, 500);
            chassis.waitUntilDone();
            chassis.moveToPoint(33, 15, 500, {.forwards = false});
            chassis.moveToPoint(36, 50, 1500, {.forwards = false});*/
            break;
        case 6:
            chassis.setPose(0,0,0);
            chassis.moveToPoint(0, 38, 2000, {.maxSpeed = 70});
            chassis.turnToHeading(90, 1000, {.maxSpeed = 80});
            
            tongue.set_value(true);
            pros::delay(750);
            firstStage.move(127);
            chassis.moveToPoint(10, 38, 1000,{.maxSpeed = 50});
            chassis.turnToHeading(90, 500);
            chassis.moveToPoint(-30, 40, 2000,{.forwards = false, .maxSpeed = 90});
            chassis.waitUntilDone();
            upperIntake.move(-127);
             pros::delay(3000);
            chassis.moveToPoint(8, 40, 1000,{.maxSpeed = 50});
            chassis.moveToPoint(-30, 40, 2000,{.forwards = false, .maxSpeed = 90});
            break;
        case 7:
            chassis.setPose(0,0,0);
            chassis.moveToPoint(0, 38, 2000, {.maxSpeed = 70});
            chassis.turnToHeading(-90, 1000, {.maxSpeed = 80});
            
            tongue.set_value(true);
            pros::delay(1250);
            firstStage.move(127);
            chassis.moveToPoint(-12, 38, 700,{.maxSpeed = 50});
            chassis.turnToHeading(-90, 500);
            chassis.moveToPoint(30, 40, 1000,{.forwards = false, .maxSpeed = 90});
            chassis.waitUntilDone();
            upperIntake.move(-127);
            pros::delay(2500);
            upperIntake.move(0);
            chassis.waitUntilDone();
            tongue.set_value(false);
            chassis.moveToPoint(0, 40, 1000,{.maxSpeed = 70});
            chassis.turnToHeading(133, 500,{.maxSpeed = 80});
            chassis.moveToPoint(25, 15, 1600,{.maxSpeed = 60});
            chassis.waitUntilDone();
            tongue.set_value(true); 
            pros::delay(500);
            //dump 2nd batch
            chassis.moveToPoint(0, 44, 1000,{.forwards = false, .maxSpeed = 70});
            chassis.turnToHeading(-90, 500,{.maxSpeed = 70});
            chassis.moveToPoint(30, 40, 1000,{.forwards = false, .maxSpeed = 90});
            chassis.waitUntilDone();

            firstStage.move(127);
            upperIntake.move(-127);
            
            break;
        case 8: 
            chassis.setPose(0,0,0);
            firstStage.move(127);
            chassis.moveToPoint(0, 21, 1000);
            chassis.turnToHeading(-30, 250);
            chassis.moveToPoint(-10, 40, 1000, {.maxSpeed = 50});
            chassis.moveToPoint(-28, 46, 1000);
            chassis.waitUntilDone();
            // tongue.set_value(true);
            pros::delay(100);
            chassis.moveToPoint(-10, 25, 1000, {.forwards = false});
            chassis.turnToHeading(215, 500);
            firstStage.move(0);
            chassis.moveToPoint(-32, 15, 1000);
            chassis.turnToHeading(180, 500);
            chassis.moveToPoint(-32, 40, 1000, {.forwards = false});
            chassis.waitUntilDone();
            //firstStage.move(127);
            //hood.set_value(true);
            //upperIntake.move(-127);
            unload =true;
            pros::delay(1500);
            unload = false;
            firstStage.move(0);
            upperIntake.move(0);
            hood.set_value(false);
            chassis.setPose(0, 0, (imu1.get_heading()+180));
            chassis.moveToPoint(-2, 20, 500);
            tongue.set_value(true);
            firstStage.move(127);
            chassis.moveToPoint(-2, 32, 1100, {.maxSpeed = 70});
            chassis.waitUntilDone();
            chassis.moveToPoint(1, -10, 1000, {.forwards = false});
            chassis.waitUntilDone();
            //hood.set_value(true);
            //upperIntake.move(-127);
            unload = true;
            pros::delay(1500);
            //hood.set_value(false);
            unload = false;
            chassis.moveToPoint(1, 15, 500);
            chassis.moveToPoint(1, -20, 500, {.forwards = false});
            break;
        case 9:
            chassis.setPose(0,0,0);
            firstStage.move(127);
            chassis.moveToPoint(0, 21, 1000);
            chassis.turnToHeading(-30, 250);
            chassis.moveToPoint(10, 40, 1000, {.maxSpeed = 50});
            chassis.moveToPoint(28, 46, 1000);
            chassis.waitUntilDone();
            pros::delay(100);
            chassis.moveToPoint(10, 25, 1000, {.forwards = false});
            chassis.turnToHeading(-215, 500);
            firstStage.move(0);
            chassis.moveToPoint(33, 15, 1000);
            chassis.turnToHeading(-180, 500);
            chassis.moveToPoint(33, 40, 1000, {.forwards = false});
            chassis.waitUntilDone();
            firstStage.move(127);
            hood.set_value(true);
            upperIntake.move(-127);
            pros::delay(1500);
            firstStage.move(0);
            upperIntake.move(0);
            hood.set_value(false);
            chassis.setPose(0, 0, (imu1.get_heading()+180));
            chassis.moveToPoint(2, 15, 1000);
            tongue.set_value(true);
            firstStage.move(127);
            chassis.moveToPoint(2, 32, 1000, {.maxSpeed = 50});
            chassis.waitUntilDone();
            pros::delay(400);
            chassis.moveToPoint(0, -5, 1000, {.forwards = false});
            chassis.waitUntilDone();
            hood.set_value(true);
            upperIntake.move(-127);
            pros::delay(1500);
            hood.set_value(false);
            chassis.moveToPoint(0, 15, 500);
            chassis.moveToPoint(0, -20, 500, {.forwards = false});
            break;
        case 10:
            //prepare to collect from holder
            chassis.setPose(0,0,0);
            chassis.moveToPoint(0, 39, 1000, {.maxSpeed = 100});
            chassis.waitUntilDone();
            tongue.set_value(true);
            chassis.turnToHeading(90, 500, {.maxSpeed = 127});
            chassis.waitUntilDone();
            load = true;
            chassis.moveToPoint(14, 39, 2000,{.maxSpeed = 70});
            chassis.waitUntilDone();
            chassis.moveToPoint(-30, 40, 1000,{.forwards = false, .maxSpeed = 100});
            chassis.waitUntilDone();
            load = false;
            unload = true;
            pros::delay(3000);
            chassis.waitUntilDone();
            tongue.set_value(false);
            chassis.turnToHeading(90, 500);
            chassis.waitUntilDone();
            chassis.setPose(0,0,imu1.get_heading());
            // after scoring get 2 blocks in 1st quad
            chassis.moveToPoint(20, 0, 1000);
            chassis.waitUntilDone();
            load = true;
            unload = false;
            chassis.turnToHeading(180, 750);
            chassis.waitUntilDone();
            chassis.moveToPoint(20, -23, 1000);
            chassis.waitUntilDone();
            chassis.turnToHeading(270, 750);
            chassis.waitUntilDone();
            chassis.moveToPoint(4, -22.5, 1000);
            chassis.waitUntilDone();
            pros::delay(250);
            chassis.moveToPoint(14, -23, 750, {.forwards = false});
            chassis.waitUntilDone();
            chassis.turnToHeading(0, 750);
            chassis.waitUntilDone();
            chassis.moveToPoint(14, 2, 1000);
            chassis.waitUntilDone();
            chassis.turnToHeading(90, 750);
            chassis.waitUntilDone();
            chassis.moveToPoint(-5, 1, 1000, {.forwards = false, .maxSpeed = 50});
            chassis.waitUntilDone();
            load = false;
            unload = true;
            pros::delay(1250);
            chassis.turnToHeading(90, 500);
            chassis.waitUntilDone();
            chassis.setPose(0,0,imu1.get_heading());
            pros::delay(100);
            // move to other match load tube
            chassis.moveToPoint(15, 0, 1000);
            chassis.waitUntilDone();
            unload = false;
            chassis.turnToHeading(180, 750);
            chassis.waitUntilDone();
            chassis.moveToPoint(10, -93, 3500, {.maxSpeed = 70});
            chassis.waitUntilDone();
            chassis.turnToHeading(90, 750);
            tongue.set_value(true);
            load = true;
            chassis.waitUntilDone();
            chassis.moveToPoint(26, -94, 2000, {.maxSpeed = 70});
            chassis.waitUntilDone();
            chassis.moveToPoint(-15, -94, 1000, {.forwards = false, .maxSpeed = 100});
            chassis.waitUntilDone();
            load = false;
            unload = true;
            pros::delay(3000);
            chassis.turnToHeading(90, 500);
            chassis.waitUntilDone();
            chassis.setPose(0,0,imu1.get_heading());
            pros::delay(100);
            // get other 2 blocks
            chassis.moveToPoint(20, 0, 1000);
            chassis.waitUntilDone();
            tongue.set_value(false);
            unload = false;
            load = true;
            chassis.turnToHeading(0, 750);
            chassis.waitUntilDone();
            chassis.moveToPoint(20, 24, 1000);
            chassis.waitUntilDone();
            chassis.turnToHeading(270, 750);
            chassis.waitUntilDone();
            chassis.moveToPoint(5, 24, 1000);
            chassis.waitUntilDone();
            pros::delay(250);
            chassis.moveToPoint(15, 24, 1000, {.forwards = false});
            chassis.waitUntilDone();
            chassis.turnToHeading(180, 750);
            chassis.waitUntilDone();
            chassis.moveToPoint(15, 0, 1000);
            chassis.waitUntilDone();
            chassis.turnToHeading(90, 750);
            chassis.waitUntilDone();
            chassis.moveToPoint(-5, 0, 1000, {.forwards = false, .maxSpeed = 50});
            chassis.waitUntilDone();
            load = false;
            unload = true;
            pros::delay(1250);
            chassis.turnToHeading(90, 500);
            chassis.waitUntilDone();
            chassis.setPose(0,0,imu1.get_heading());
            pros::delay(100);
            // park and clear
            // chassis.moveToPoint(35, 27, 1000, {.maxSpeed = 90});
            // chassis.waitUntilDone();
            // unload = false;
            // chassis.turnToHeading(0, 750);
            // chassis.waitUntilDone();
            // load = true;
            // chassis.moveToPoint(37, 45, 2000, {.maxSpeed = 80});

            // tongue mech clear
            chassis.moveToPoint(25, 0, 1000);
            chassis.waitUntilDone();
            chassis.turnToHeading(45, 750);
            chassis.waitUntilDone();
            chassis.moveToPoint(37, 50, 3000);
            pros::delay(570);
            tongue.set_value(true);
            chassis.waitUntilDone();
            tongue.set_value(false);
    }
    
}
int avg(int arr[10]) {
int c = 0;
for (int i =0; i < 10; i++) {
    c+= arr[i];
}
return c/10;
}
void extake() {
    int prev[10];
    int count = 0;
    firstStage.move(127);
    upperIntake.move(-127);
    //while (std::abs(firstStage.get_actual_velocity()) < 600 || std::abs(upperIntake.get_actual_velocity()) < std::abs(firstStage.get_actual_velocity()) < 20 || std::abs(upperIntake.get_actual_velocity()) < 20)
    while (avg(prev) < 600) {
        prev[count] = std::min(std::abs(firstStage.get_actual_velocity()), std::abs(upperIntake.get_actual_velocity()));
        if (count==9) {
            count = 0;
        }
        else {
            count++;
        }
        pros::delay(50);
    }
    firstStage.move(0);
    upperIntake.move(0);
}
void opcontrol() {
    bool tongueOut = false;
    bool wingsOut = false;
    bool doubleOut = false;
    bool gatesOut = false;

    // loop forever
    while (true) {
        
        // Exponential drive control
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = (controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X))/1.2;

		double cubedLeftY = (leftY * leftY * leftY);
		double cubedRightX = (rightX * rightX * rightX);

		double expY = (cubedLeftY/20000);
		double expX = (cubedRightX/20000);

		double expL = (leftY + rightX);
		double expR = (leftY - rightX);

		leftMotors.move(expL);
        rightMotors.move(expR);
        
		// intake
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) { // score on high goal
            /*hood.set_value(true);
			firstStage.move(127);
			upperIntake.move(-127);*/
            unload = true;
            //extake();
		} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) { // first stage only intake
            hood.set_value(false);
            midGoal.set_value(false);
            //firstStage.move(127);
            //upperIntake.move(-127);
            load = true;

		} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) { // first stage only intake
			firstStage.move(127);
            upperIntake.move(-50);
            
		}else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
            // first stage only extake
            hood.set_value(false);
			firstStage.move(-127);
            upperIntake.move(127);
            midGoal.set_value(false);
            //firstStage.move(127);
		} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            hood.set_value(false); // first stage only extake
			firstStage.move(127);
            upperIntake.move(70);
            midGoal.set_value(true);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
            midGoal.set_value(true);

        } else { // stop both
			firstStage.move(0);
			upperIntake.move(0);
            load = false;
            unload = false;
		}


        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
			tongueOut = !tongueOut;
            pros::delay(250);
			tongue.set_value(tongueOut);
		}

        // if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
		// 	wingsOut = !wingsOut;
        //     pros::delay(250);
		// 	wings.set_value(wingsOut);
		// }

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
            wingsOut = !wingsOut;
            pros::delay(250);
            wings.set_value(wingsOut);
        }
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)){
            gatesOut = !gatesOut;
            pros::delay(250);
            midGoal.set_value(gatesOut);
        }
        // delay to save resources
        pros::delay(20);
    }
}
