#include "main.h"
#include "pros/misc.h"
#include "pros/motors.hpp"
#include "lemlib/api.hpp" // IWYU pragma: keep

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::MotorGroup leftMotors({11 , -15, -13}, pros::MotorGearset::blue); // left motors on ports 1, 2, 3
pros::MotorGroup rightMotors({-20, 19,18}, pros::MotorGearset::blue); // right motors on ports 4, 5, 6
//first = front
// other motors
pros::Motor firstStage(14, pros::MotorGearset::blue);
pros::Motor upperIntake(2, pros::MotorGearset::blue);

// pnaumatics
pros::adi::DigitalOut tongue('B');
pros::adi::DigitalOut wings('C');

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              11, // 10 inch track width
                              lemlib::Omniwheel::NEW_275, // using new 4" omnis
                              450, // drivetrain rpm is 360
                              0 // horizontal drift is 2 (for now)
);
// create an imu on port 10
pros::Imu imu1(3);
pros::Imu imu2(4);
// create a v5 rotation sensor on port 1
pros::Rotation vertical_encoder(-10);
// horizontal tracking wheel
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_275,0);
lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            nullptr, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu1 // inertial sensor
);

// lateral PID controller
 lemlib::ControllerSettings lateral_controller(7, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              5, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                            10 // maximum acceleration (slew)
);

// angular PID controller
//tried 6.6 and greater and none worked 
//6.5 40 --to within +-0.2 degrees @90 degree clocwise turn
lemlib::ControllerSettings angular_controller(5, // proportional gaisn (kPs)
                                              0, // integral gain (kI)
                                              46, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// create the chassis
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);

void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    // print position to brain screen
    pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // delay to save resources
            pros::delay(20);
        }
    });
}

/* 
    0 - lateral pid test
    1 - angular pid test
    2 - 
    3 - old skills (dont use)
    4 - left match
    5 - skills
    6 - right match
*/
int chosenAuton = 7;

void autonomous() {
    switch(chosenAuton){
		// lateral pid test
        case 0:
            chassis.moveToPoint(0, 20, 2000);
            chassis.waitUntilDone();
            chassis.moveToPoint(0, 0, 2000, {.forwards = false});

            // end
            chassis.waitUntilDone();
            break;
        // angular pid test
        case 1:
            chassis.turnToHeading(90, 2000);
            chassis.waitUntilDone();
            chassis.turnToHeading(0, 2000);

            // end
            chassis.waitUntilDone();
            break;
        // simple match auto - aneek makes
        case 2:
            break;
        
        case 3:
            // intake first blocks
            chassis.moveToPoint(0, 23, 1000);
            chassis.waitUntilDone();
            chassis.turnToHeading(90, 750);
            firstStage.move(127);
            chassis.waitUntilDone();
            chassis.moveToPoint(17, 22, 1500, {.maxSpeed = 30});
            chassis.waitUntilDone();

            // score them on bottom goal
            chassis.turnToHeading(-45, 750);
            chassis.waitUntilDone();
            chassis.moveToPoint(5, 36, 1000);
            chassis.waitUntilDone();
            firstStage.move(-127);
            pros::delay(2000);
            chassis.moveToPoint(15, 20, 1000, {.forwards = false});
            chassis.waitUntilDone();

            // descore and score
            chassis.turnToHeading(135, 750);
            chassis.waitUntilDone();
            chassis.moveToPoint(41, 3, 2000);
            chassis.waitUntilDone();
            tongue.set_value(true);
            chassis.turnToHeading(180, 750);
            chassis.waitUntilDone();
            chassis.moveToPoint(41, -10, 1000);
            chassis.waitUntilDone();
            pros::delay(1000);
            chassis.moveToPoint(41, 20, 1000);
            chassis.waitUntilDone();
            upperIntake.move(-127);

            break;
        case 4:
            chassis.setPose(0,0,0);
            chassis.moveToPoint(0, 38, 2000, {.maxSpeed = 70});
            chassis.turnToHeading(-90, 1000, {.maxSpeed = 80});
            
            tongue.set_value(true);
            pros::delay(1250);
            firstStage.move(127);
            chassis.moveToPoint(-12, 38, 1100,{.maxSpeed = 50});
            chassis.turnToHeading(-90, 500);
            chassis.moveToPoint(30, 40, 1000,{.forwards = false, .maxSpeed = 90});
            chassis.waitUntilDone();
            upperIntake.move(-127);
            pros::delay(2500);
            upperIntake.move(0);
            chassis.waitUntilDone();
            tongue.set_value(false);
            chassis.moveToPoint(0, 40, 1000,{.maxSpeed = 60});/*
            chassis.moveToPoint(30, 40, 1000,{.forwards = false, .maxSpeed = 90});*/
            chassis.turnToHeading(133, 1000,{.maxSpeed = 80});
            chassis.moveToPoint(28, 13, 1400,{.maxSpeed = 60});
            chassis.waitUntilDone();
            tongue.set_value(true);
            chassis.turnToHeading(-47, 800,{.maxSpeed = 70});
            chassis.moveToPoint(37, 5, 1250, {.forwards = false, .maxSpeed = 50});
            chassis.turnToHeading(-54, 200);
            chassis.waitUntilDone();
            upperIntake.move(-70);
            break;

        case 5://prepare to collect from holder
            chassis.setPose(0,0,0);
            chassis.moveToPoint(0, 39, 1500, {.maxSpeed = 70});
            chassis.turnToHeading(-90, 500, {.maxSpeed = 80});
            tongue.set_value(true);
            pros::delay(500);
            //collect balls
            firstStage.move(127);
            chassis.moveToPoint(-10.3, 39, 2000,{.maxSpeed = 50});
            chassis.turnToHeading(-90, 500);
            chassis.waitUntilDone();
            firstStage.move(0);
            //dump balls
            chassis.moveToPoint(25, 40, 1000,{.forwards = false, .maxSpeed = 80});
            chassis.waitUntilDone();
            firstStage.move(127);
            upperIntake.move(-127);
            pros::delay(1000);
            firstStage.move(-127);
            pros::delay(250);
            firstStage.move(127);
            pros::delay(2000);
            chassis.waitUntilDone();
            tongue.set_value(false);
            upperIntake.move(0);
            //pick up 2nd batch
            chassis.moveToPoint(0, 40, 1000,{.maxSpeed = 70});
            chassis.turnToHeading(133, 500,{.maxSpeed = 80});
            chassis.moveToPoint(25, 15, 1400,{.maxSpeed = 60});
            chassis.waitUntilDone();
            tongue.set_value(true); 
            pros::delay(500);
            //dump 2nd batch
            firstStage.move(0);
            chassis.moveToPoint(0, 44, 1000,{.forwards = false, .maxSpeed = 70});
            chassis.turnToHeading(-90, 500,{.maxSpeed = 70});
            chassis.moveToPoint(30, 40, 1000,{.forwards = false, .maxSpeed = 90});
            chassis.waitUntilDone();
            firstStage.move(127);
            upperIntake.move(-127);
            pros::delay(500);
            firstStage.move(-127);
            pros::delay(250);
            firstStage.move(127);
            pros::delay(1000);
            upperIntake.move(-127);
            firstStage.move(-127);
            tongue.set_value(false); 
            //orient
            chassis.moveToPoint(10, 40, 1000, {.maxSpeed = 70});
            chassis.turnToHeading(-173, 500);
            chassis.moveToPoint(10, 20, 1000, {.maxSpeed = 70});
            chassis.waitUntilDone();
            chassis.turnToHeading(90, 500);
            chassis.moveToPoint(85, 25, 2500);
            //turn and pick up
            chassis.turnToHeading(0,500);
            chassis.waitUntilDone();
            chassis.moveToPoint(85, 43, 1000, {.maxSpeed = 70});
            chassis.turnToHeading(90, 500);
            firstStage.move(127);
            upperIntake.move(0);
            tongue.set_value(true);
            chassis.waitUntilDone();
            //chassis.moveToPoint(90, 40, 1000, {.maxSpeed = 70});
            //chassis.waitUntilDone();
            //chassis.turnToHeading(90, 750);
            chassis.moveToPoint(105, 43, 3500, {.maxSpeed = 50});
            chassis.waitUntilDone();
            chassis.turnToHeading(90, 500);
            chassis.waitUntilDone();
            firstStage.move(0);
            chassis.moveToPoint(60, 40, 1000, {.forwards = false, .maxSpeed = 90});
            chassis.waitUntilDone();
            firstStage.move(127);
            upperIntake.move(-127);
            pros::delay(1000);
            firstStage.move(-127);
            pros::delay(250);
            firstStage.move(127);
            pros::delay(2000);
            tongue.set_value(false);
            upperIntake.move(-127);
            firstStage.move(-127);
            //move to third quad
            tongue.set_value(false);
            chassis.setPose(60,39,90);
            chassis.moveToPoint(70, 40, 1000, {.maxSpeed = 70});
            chassis.waitUntilDone();
            chassis.turnToHeading(180, 750);
            chassis.waitUntilDone();
            chassis.moveToPoint(80, -50, 2500, {.maxSpeed = 100});
            chassis.waitUntilDone();
            chassis.turnToHeading(90, 500);
            chassis.waitUntilDone();
            //dump
            tongue.set_value(true);
            firstStage.move(127);
            upperIntake.move(0);
            chassis.moveToPoint(89, -50, 3000, {.maxSpeed = 50});
            chassis.turnToHeading(90, 500);
            chassis.waitUntilDone();
            firstStage.move(0);
            chassis.waitUntilDone();
            chassis.moveToPoint(55, -40, 2000, {.forwards = false, .maxSpeed = 70});
            chassis.waitUntilDone();
            firstStage.move(127);
            upperIntake.move(-127);
            pros::delay(1000);
            firstStage.move(-127);
            pros::delay(250);
            firstStage.move(127);
            pros::delay(2000);
            upperIntake.move(-127);
            tongue.set_value(false);
            firstStage.move(-127);
            //mov3 4th quad
            chassis.moveToPoint(70, -55, 1000);
            chassis.waitUntilDone();
            chassis.turnToHeading(0,500);
            chassis.waitUntilDone();
            chassis.moveToPoint(70, -20, 1000);
            chassis.waitUntilDone();
            chassis.turnToHeading(-90,500);
            chassis.waitUntilDone();
            chassis.moveToPoint(-5, -20, 2500);
            chassis.waitUntilDone();

            chassis.turnToHeading(180,500);
            chassis.waitUntilDone();
            chassis.moveToPoint(-5, -55, 1000);
            chassis.waitUntilDone();
            chassis.turnToHeading(-90,500);
            tongue.set_value(true);
            firstStage.move(127);
            upperIntake.move(-127); // (if doing park keep, if scoring then comment out)
            chassis.waitUntilDone();
            chassis.moveToPoint(-26, -56, 3000, {.maxSpeed = 50});
            chassis.waitUntilDone();
            // park
            firstStage.move(-127);
            chassis.moveToPoint(-10, -57, 1000, {.forwards = false});
            chassis.waitUntilDone();
            tongue.set_value(false);
            firstStage.move(127);
            chassis.turnToHeading(-60, 500);
            chassis.moveToPoint(-37, -5, 3000);

            // // score in goal (only use as backup if park no work)
            // chassis.moveToPoint(10, -57, 2000, {.forwards=false});
            // chassis.waitUntilDone();
            // upperIntake.move(-127);
            // pros::delay(1000);
            // firstStage.move(-127);
            // pros::delay(250);
            // firstStage.move(127);
            // pros::delay(2000);
            // upperIntake.move(0);
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
    }
    
}

void opcontrol() {
    bool tongueOut = false;
    bool wingsOut = false;

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
			firstStage.move(127);
			upperIntake.move(-127); 
		} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) { // first stage only intake
			firstStage.move(127);
		} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) { // first stage only intake
			firstStage.move(127);
            upperIntake.move(-90);
		}else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) { // first stage only extake
			firstStage.move(-127);
            upperIntake.move(127);
		} else { // stop both
			firstStage.move(0);
			upperIntake.move(0);
		}

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
			tongueOut = !tongueOut;
			pros::delay(250);
			tongue.set_value(tongueOut);
		}

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
			wingsOut = !wingsOut;
			pros::delay(250);
			wings.set_value(wingsOut);
		}

        // delay to save resources
        pros::delay(25);
    }
}
