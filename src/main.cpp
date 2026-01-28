#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/adi.hpp" 
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"

bool pRon = false;
bool pYon = false;
bool pL2on = false;

bool pR_prev = false;
bool pY_prev = false;
bool pL2_prev = false;

int autonselected = 0;

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup rightMotors({5, 6, -7}, pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup leftMotors({-8, -9, 4}, pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)

pros::Motor bottom_intake(-1, pros::MotorGearset::blue); // intake motors on ports 7 and 20

pros::Motor top_intake(2, pros::MotorGearset::blue);

// Define pneumatics
pros::adi::DigitalOut piston1('A'); // piston on port A, score
pros::adi::DigitalOut piston2('B'); // piston on port B, match loader
pros::adi::DigitalOut piston3('C'); // piston on port C, doinker

// Inertial Sensor on port 10
pros::Imu imu(10);

// auton selector port D
pros::adi::AnalogIn autonSelector('D');

// tracking wheels
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
pros::Rotation verticalEnc(-20);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_2, 2.5);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              12, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, 
                              450, 
                              2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              5, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              1 // maximum acceleration (slew)
);

lemlib::ControllerSettings angular_controller(4, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              18, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            nullptr, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttle_curve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steer_curve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller, sensors, &throttle_curve, &steer_curve);

// auton 1 made by rishi and parth, right
void auton1() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    //set position to x:0, y:0, heading:0
    chassis.setPose(0, 0, 0);
    //set for high goal
    piston2.set_value(true);
    piston3.set_value(true);
    // turn to face heading 90 with a very long timeout
    bottom_intake.move_velocity(600);
    chassis.moveToPose(10.174, 34.154, 20.36, 2000, {.maxSpeed = 300});
    chassis.waitUntilDone();
    // move to match loader ready position
    chassis.turnToHeading(120, 2000, {.maxSpeed = 300});
    chassis.waitUntilDone();
    chassis.moveToPoint(40.2556,7, 2000, {.maxSpeed = 50});
    chassis.waitUntilDone();
    chassis.turnToHeading(180, 2000, {.maxSpeed = 300});
    chassis.waitUntilDone();
    piston1.set_value(true);
    pros::delay(1000);
    chassis.moveToPoint(40.2556, -10.8133, 2000, {.minSpeed = 200});
    chassis.waitUntilDone();
    drivetrain.leftMotors->move_velocity(28);
    drivetrain.rightMotors->move_velocity(28);
    pros::delay(1750);
    drivetrain.leftMotors->move_velocity(0);
    drivetrain.rightMotors->move_velocity(0);
    chassis.moveToPose(42,23, 180, 2000, {.forwards = false, .maxSpeed = 100});
    chassis.waitUntilDone();
    top_intake.move_velocity(600);   
}

// check auton button press
void checkAutonButton() {
    static bool laststate = false;
    while (true) {
        bool currentstate = autonSelector.get_value() > 2000;
        if (currentstate && !laststate) {
            autonselected = (autonselected + 1) % 2; // modulo, replace last num with amount of autons
        }
        laststate = currentstate;
        pros::delay(20);
    }
}

void selectAuton() {
    if (autonselected == 0) {
        auton1();
    } else if (autonselected == 1) {
        // fart
    }
}
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            controller.print(0, 0, "Intake Temp: %f", pros::c::motor_get_temperature(1)); // x            // log position telemetry
            controller.print(1, 0, "Auton: %d", autonselected);
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(100);
        }
    });
    // start auton selector button checking task
    pros::Task autonButtonTask(checkAutonButton);
}

/**
 * Runs while the robot is disabled
 */
void disabled() {
    // KADEN: put auton selector code here
    // nah
    // i have made a severe and continuous lapse in my judgement and i do not expect to be forgiven. - logan paul
}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(example_txt); // '.' replaced with "_" to make c++ happy

/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
void autonomous() {
    selectAuton();
}  

/**
 * Runs in driver control
 */
void opcontrol() {
    pRon = true;
    pYon = true;
    pL2on = true;
    pR_prev = true;
    pY_prev = true;
    pL2_prev = true;
    // controller
    // loop to continuously update motors
    while (true) {
        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int leftX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
		bool intakeForwardButton = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
		bool intakeBackwardButton = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
        bool flapButton = controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y);
		bool pR = controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT);	// score
		bool pY = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1); // match loader
        bool pL2 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2); // doinker

        // move the robot
        chassis.arcade(leftY, leftX, false, 0.7);

        // control the intake motors
        if (intakeForwardButton) {
            bottom_intake.move_velocity(590);
        } else if (intakeBackwardButton) {
            bottom_intake.move_velocity(590);
            top_intake.move_velocity(590);
        } else if (flapButton) {
            top_intake.move_velocity(-590);
            bottom_intake.move_velocity(-590);
        } else {
            bottom_intake.move_velocity(0);
            top_intake.move_velocity(0);
        }

		// control the pistons
		// X button toggle
		if (pR && !pR_prev) {           // just pressed this loop
			pRon = !pRon;               // flip state
			piston1.set_value(pRon);   // set piston to state
		}

		// B button toggle
		if (pY && !pY_prev) {           // just pressed this loop
			pYon = !pYon;
			piston2.set_value(pYon);
		}

        if (pL2 && !pL2_prev) {           // just pressed this loop
            pL2on = !pL2on;
            piston3.set_value(pL2on);
        }

		// remember button state for next loop
		pR_prev = pR;
		pY_prev = pY;
        pL2_prev = pL2;

        // delay to save resources
        pros::delay(10);
    }
}
