#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep

<<<<<<< HEAD
bool Xon = false;
bool Bon = true;

bool X_prev = false;
bool B_prev = false;

=======
>>>>>>> 1a7fc88e2d1d1fb255d73ee99b65547512259561
// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup rightMotors({5, 6, -7},
                            pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup leftMotors({-8, -9, 4}, pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)
<<<<<<< HEAD

pros::Motor bottom_intake(-1, pros::MotorGearset::blue); // intake motors on ports 7 and 20

pros::Motor top_intake(2, pros::MotorGearset::blue);

// Define pneumatics
pros::adi::DigitalOut piston1('A'); // piston on port B
pros::adi::DigitalOut piston2('B'); // piston on port C

=======

>>>>>>> 1a7fc88e2d1d1fb255d73ee99b65547512259561
// Inertial Sensor on port 10
pros::Imu imu(10);

// tracking wheels
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
pros::Rotation verticalEnc(20);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
<<<<<<< HEAD
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_2, 2.5);
=======
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_2, 0.75);
>>>>>>> 1a7fc88e2d1d1fb255d73ee99b65547512259561

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
<<<<<<< HEAD
                              12, // 10 inch track width
=======
                              12.25, // 10 inch track width
>>>>>>> 1a7fc88e2d1d1fb255d73ee99b65547512259561
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              450, // drivetrain rpm is 360
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
                                              0 // maximum acceleration (slew)
);

<<<<<<< HEAD
lemlib::ControllerSettings angular_controller(4, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              18, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
=======
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              100, // integral gain (kI)
                                              20, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
>>>>>>> 1a7fc88e2d1d1fb255d73ee99b65547512259561
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
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
<<<<<<< HEAD
lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller, sensors, &throttle_curve, &steer_curve);
=======
lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller, sensors, &throttleCurve, &steerCurve);
>>>>>>> 1a7fc88e2d1d1fb255d73ee99b65547512259561

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors

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
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });
}

/**
 * Runs while the robot is disabled
 */
void disabled() {}

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
<<<<<<< HEAD
    // set position to x:0, y:0, heading:0
    chassis.setPose(0, 0, 0);
    //set for high goal
    piston2.set_value(true);
    // turn to face heading 90 with a very long timeout
    bottom_intake.move_velocity(600);
    top_intake.move_velocity(-600);
    chassis.moveToPose(6.32, 26.23, 22.08, 5000, {.maxSpeed = 300});
    // move to match loader ready position
    chassis.turnToHeading(120, 5000, {.maxSpeed = 300});
    chassis.moveToPoint(30.34,1.286, 5000, {.maxSpeed = 300});
    chassis.turnToHeading(180, 5000, {.maxSpeed = 300});
    piston1.set_value(true);
    pros::delay(500);
    chassis.moveToPose(30.34, -8, 179.0, 5000, {.maxSpeed = 300});
    chassis.moveToPose(30.834, 22.904, 178.05, 5000, {.forwards = false, .maxSpeed = 30});
    
}
=======
   // set position to x:0, y:0, heading:0
    chassis.setPose(0, 0, 0);
    // turn to face heading 90 with a very long timeout
    chassis.turnToHeading(90, 100000);}
>>>>>>> 1a7fc88e2d1d1fb255d73ee99b65547512259561

/**
 * Runs in driver control
 */
void opcontrol() {
    // controller
    // loop to continuously update motors
    while (true) {
        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
<<<<<<< HEAD
		bool intakeForwardButton = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
		bool intakeBackwardButton = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
		bool X = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);		
		bool B = controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y);

        // move the robot
        chassis.curvature(leftY, rightX);

        // control the intake motors
        if (intakeForwardButton) {
            bottom_intake.move_velocity(600);
        } else if (intakeBackwardButton) {
            bottom_intake.move_velocity(-600);
        } else {
            bottom_intake.move_velocity(0);
        }

		// control the pistons
		// X button toggle
		if (X && !X_prev) {           // just pressed this loop
			Xon = !Xon;               // flip state
			piston1.set_value(Xon);   // set piston to state
		}

		// B button toggle
		if (B && !B_prev) {           // just pressed this loop
			Bon = !Bon;
			piston2.set_value(Bon);
		}

		// remember button state for next loop
		X_prev = X;
		B_prev = B;

=======
        // move the chassis with curvature drive
        chassis.arcade(leftY, rightX);
>>>>>>> 1a7fc88e2d1d1fb255d73ee99b65547512259561
        // delay to save resources
        pros::delay(10);
    }
<<<<<<< HEAD
}
=======
}
>>>>>>> 1a7fc88e2d1d1fb255d73ee99b65547512259561
