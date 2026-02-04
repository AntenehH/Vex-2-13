#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup leftMotors({-12, -13, -14},
pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup rightMotors({1, 2, 3}, pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)

// Inertial Sensor on port 10
pros::Imu imu(10);
pros::ADIDigitalOut switchPiston('B');
pros::ADIDigitalOut matchLoader('A');
pros::ADIDigitalOut descore('E');
pros::Motor intakeMotor(20);
pros::Motor outTakeMotor(9);


bool inverse = false;
bool switchPistonState = false;
bool matchLoaderState = false;
bool descoreState = false;

// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
pros::Rotation horizontalEnc(-18);
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
pros::Rotation verticalEnc(-6);
// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_275, -5.75);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_275, -2.5);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              10, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              360, // drivetrain rpm is 360
                              2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(10, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            3, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(2, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             10, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            nullptr, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */


void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    lemlib::Pose poseA(53,-8,247);
    chassis.calibrate(); // calibrate sensors
    chassis.setPose(poseA);

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

void spinIntake() {
    intakeMotor.move(-127);
    outTakeMotor.move(127);
}

void stopIntake() {
    intakeMotor.move(0);
    outTakeMotor.move(0);
}

void autonomous() {
    // Move to x: 20 and y: 15, and face heading 90. Timeout set to 4000 ms
    lemlib::Pose poseA(48.5,-10.2,246);
    switchPiston.set_value(true);
    chassis.setPose(poseA);

    chassis.turnToPoint(13, -25, 1000);
    spinIntake();
    chassis.moveToPoint(13,-25, 3000, {.maxSpeed = 70});
    pros::delay(1500);
    stopIntake();
    
    chassis.turnToPoint(30, -48.5, 1000);
    chassis.moveToPoint(30, -48.5, 1000);
    
    chassis.turnToPoint(55, -54, 2000);
    chassis.moveToPoint(55, -54, 2000);

    chassis.moveToPoint(19.3, -54, 2000, {.forwards = false, .earlyExitRange = 1});
    
    pros::lcd::print(4, "Traveled 10 inches during pure pursuit!");
    // wait until the movement is done
    pros::lcd::print(4, "pure pursuit finished!");
    
}

/**
 * Runs in driver control
 */
void opcontrol() {
    // controller
    // loop to continuously update motors
    //autonomous();

    while (true) {
        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        //r1 outtake
        //l2 reverse intake
        //a middle piston
        //b for match loader
        //right arrow descorer
        if (controller.get_digital(DIGITAL_R2)) {
            spinIntake();
        } else if (controller.get_digital(DIGITAL_R1)) {
            outTakeMotor.move(-127);
        } else if (controller.get_digital(DIGITAL_L2)) {
            intakeMotor.move(127);
        } else {
            stopIntake();
        }

        // toggle match loader on B press
        if (controller.get_digital_new_press(DIGITAL_B)) {
            matchLoaderState = !matchLoaderState;
            matchLoader.set_value(matchLoaderState);
        }

        // toggle descore on RIGHT press
        if (controller.get_digital_new_press(DIGITAL_RIGHT)) {
            descoreState = !descoreState;
            descore.set_value(descoreState);
        }

        // toggle switch piston on A press
        if (controller.get_digital_new_press(DIGITAL_A)) {
            switchPistonState = !switchPistonState;
            switchPiston.set_value(switchPistonState);
        }
        

        // manual arcade with left motor compensation
        // adjust leftScale until it drives straight (increase if still curving left)
        double leftScale = 1;
        double rightScale = 0.8;
        int leftPower = (int)((leftY + rightX) * leftScale);
        int rightPower = (int)((leftY - rightX) * rightScale);
        leftMotors.move(leftPower);
        rightMotors.move(rightPower);

        // delay to save resources
        pros::delay(10);
    }
}