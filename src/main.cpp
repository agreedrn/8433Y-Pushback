#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/adi.hpp"

bool Xon = false;
bool Bon = true;

bool X_prev = false;
bool B_prev = false;

// Define controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// Define drivetrain motors
pros::MotorGroup left_motors({-2, 3, -5}, pros::MotorGearset::blue); // left motors on ports 2, 3, 5
pros::MotorGroup right_motors({1, 4, 6}, pros::MotorGearset::blue); // right motors on ports 1, 4, 6

// Define pneumatics
pros::adi::DigitalOut piston1('B'); // piston on port B
pros::adi::DigitalOut piston2('C'); // piston on port C

// Define intake motors
pros::MotorGroup intake_motors({-7, 20}, pros::MotorGearset::blue); // intake motors on ports 7 and 20

// create an imu on port 
pros::Imu imu(11);

// create a v5 rotation sensor on port 
pros::Rotation rotation_sensor(12);

// vertical tracking wheel encoder
pros::adi::Encoder vertical_encoder('F', 'D', true);

// vertical tracking wheel
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_275, -2.5);

// Define drivetrain
lemlib::Drivetrain drivetrain(&left_motors, // left motor group
                              &right_motors, // right motor group
                              9.5, // 9.5 inch track width
                              lemlib::Omniwheel::OLD_4, // using old 4" omnis
                              300, // drivetrain rpm is 300
                              2 // horizontal drift is 2 (for now)
);

// odometry settings
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            nullptr, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            nullptr // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
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
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors, // odometry sensors
						&throttle_curve, // throttle input curve
						&steer_curve // steer input curve
);

// initialize function. Runs on program startup
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
			pros::lcd::print(3, "Xon: %d", Xon); // piston1 state
			pros::lcd::print(4, "Bon: %d", Bon); // piston2 state
            // delay to save resources
            pros::delay(20);
        }
    });
}

void opcontrol() {
    // loop forever
    while (true) {
        // get left y and right x positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
		bool intakeForwardButton = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
		bool intakeBackwardButton = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
		bool X = controller.get_digital(pros::E_CONTROLLER_DIGITAL_X);		
		bool B = controller.get_digital(pros::E_CONTROLLER_DIGITAL_B);

        // move the robot
        chassis.curvature(leftY, rightX);

        // control the intake motors
        if (intakeForwardButton) {
            intake_motors.move_velocity(200);
        } else if (intakeBackwardButton) {
            intake_motors.move_velocity(-200);
        } else {
            intake_motors.move_velocity(0);
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

        // delay to save resources
        pros::delay(25);
    }
}

void autonomous() {
    // drive forward
    chassis.moveToPoint(600, -600, 4000); // drive to point (600, -600)
    chassis.turnToHeading(135, 4000); // turn to 135 degrees
    chassis.moveToPoint(1725, -1200, 4000); // drive to point (1725, -1200)
    chassis.turnToHeading(45, 4000); // turn to 45 degrees
}
