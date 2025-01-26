#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "robodash/api.h"


// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup leftMotors({2, 7, 1},pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup rightMotors({-4, -5, -3}, pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)

// Inertial Sensor on port 17
pros::Imu imu(17);

//Pneumatics (pros::ADIDigitalOut _NAME_ (ADI_PORT))
pros::ADIDigitalOut clamp('A');
pros::ADIDigitalOut doinker('B');

//Other motors
pros::Motor intake1(-12, pros::MotorGearset::blue);

pros::Motor lb1 (13, pros::MotorGearset::green);
pros::Motor lb2 (-14, pros::MotorGearset::green);

//rotation sensor
pros::Rotation ladyBrownRotation(-21);

pros::Distance clamp_sensor(16);

//optical
pros::Optical optical(18);

//current team color
char team_color = 'B';

bool intake_on = false;
bool reversed = false;

bool auton = false;
bool opC = false;

float targetTheta = 0;
int exitRange = 0;

bool clampOn = false;
bool current = false;

bool ringStop = false;
bool is_ring_stopped = false;

int sequenceStep = 0;
// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
pros::Rotation horizontalEnc(15);
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
pros::Rotation verticalEnc(-11);
// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_2, -6);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_275, -2.5);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              11.80, // 10 inch track width
                              lemlib::Omniwheel::NEW_275, // using new 4" omnis
                              600, // drivetrain rpm is 600
                              4 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(4.75, // proportional gain (kP)//
                                            0.5, // integral gain (kI) 
                                            3.00, // derivative gain (kD) //
                                            1.2, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            5, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            0 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(5.5, // proportional gain (kP) //5.35
                                             0, // integral gain (kI)
                                             49, // derivative gain (kD) //60.75
                                             0, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             5, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            &horizontal, // horizontal tracking wheel
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
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
bool is_stuck = false;

lemlib::PID armPID(3, 0, 4.77);               

//lemlib::PID armPID(4.25, 0, 5);

void LBpidTask(void* param) {
        float currentTheta = 0;

        float prevMotorPower = 0;

        int exitRange = 0;

        float error = 0.0f;

        pros::delay(25);

        while(true){

            if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {

                sequenceStep = (sequenceStep + 1) % 3;

                switch (sequenceStep) {
                    case 0:
                        targetTheta = 0;
                        lb1.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
                        lb2.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
                        exitRange = 20;
                        break;
                    case 1:
                        targetTheta = 7.5f;
                        lb1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
                        lb2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
                        exitRange = 0;
                        break;
                    case 9:
                        targetTheta = 60;
                        lb1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
                        lb2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
                        exitRange = 5;
                        break;
                    case 2:
                        targetTheta = 130;
                        lb1.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
                        lb2.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
                        exitRange = 40;
                        break;
                }
            }
                currentTheta = (float)ladyBrownRotation.get_angle() / 100;

                if (currentTheta > 300){
                    currentTheta = 360 - currentTheta;
                }
                // Get output from PID (Target - Actual (Accounts for gear ratio))
                

                error = targetTheta - currentTheta;
                
                if (abs(error) > exitRange) {
                    double out = armPID.update(error);
                    
                    lb1.move_voltage(out * 100);  // Output to motor
                    lb2.move_voltage(out * 100);
                } else {
                    if (sequenceStep == 2 && fabs(error) < 15){
                        sequenceStep = 0;
                        targetTheta = 0;
                        exitRange = 20;
                    }
                    lb1.brake();  // Stop the motor when within range with said brake mode
                    lb2.brake();
                }
                
                pros::delay(40);  // Don't hog the CPU
            
        } 

    
}

void is_stuck_check(void* param) {
    float intake_torque = 0;
    float angle = 0;

    while (true){
        intake_torque = intake1.get_torque();
        //pros::lcd::print(6, "%f", intake_torque);

        if (intake_torque > 0.34 && intake1.get_actual_velocity() == 0 && !reversed){
            is_stuck = true;
            intake1.move(120);
            pros::delay(200);
            intake1.move(0);
            /*if (sequenceStep == 1){
                targetTheta = 60;
                lb1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
                lb2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
                exitRange = 5;
            }*/
            
        }
        else{
            is_stuck = false;
            pros::delay(30);
        }
    }

}

void color_check(void* param) {
    // Variables for RGB values
    float hue = 0;
    int x = 0;

    while (true) {
        // Read RGB values from the optical sensor
        hue = optical.get_hue();
        pros::lcd::print(6, "%f", hue);
        
        if (!is_stuck){
            is_ring_stopped = false;
            if (((team_color == 'R' && hue > 130) || (team_color == 'B' && hue < 22))) {
                // Wrong color detected: Reject object
                pros::delay(175);
                intake1.move(120); 
                pros::delay(200);
            } 
            else if(ringStop){
                if ((team_color == 'R' && hue < 22) || (team_color == 'B' && hue > 130) || x >= 200){
                    intake1.move(0);
                    while (ringStop){
                        is_ring_stopped = true;
                        pros::delay(200);
                    }
                }
                else{
                    x++;
                    intake1.move(-95);
                }
                
            }
            else {
                // Correct color detected or no wrong color
                if (intake_on) {
                    intake1.move(-127); // Keep the intake running forward
                } else {
                    intake1.move(0);   // Stop the intake if not needed
                }
            }
        }

        if (opC){
            break;
        }

        // Small delay to avoid overwhelming the system
        pros::delay(10);
    }
}

void clampCheck(void* param){
    bool check = false;
    int distance = 100;
    

    while(true){
        distance = clamp_sensor.get();
        //pros::lcd::print(6, "%i", distance);
        if (clampOn && distance < 50 && !current){
            pros::delay(600);
            clamp.set_value(true);
            current = true;
        }
        else if (!clampOn && current){
            clamp.set_value(false);
            current = !current;
            pros::delay(10);
        }

        if (opC){
            break;
        }
        pros::delay(20);
    }
}

void best_auton() { std::cout << "Running best auton" << std::endl; }
void simple_auton() { std::cout << "Running simple auton " << std::endl; }
void good_auton() { std::cout << "Running good auton" << std::endl; }

rd::Selector selector({
    {"Best auton", &best_auton},
    {"Simple auton", &simple_auton},
    {"Good auton", &good_auton},
    {"Best auton", &best_auton},
    {"Simple auton", &simple_auton},
    {"Good auton", &good_auton},
});

rd::Console console;

rd_view_t *rd_view_create(const char *name);
void rd_view_focus(rd_view_t *name);

//rd::Console hi;

void initialize() {
    //selector.focus();
    //pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors


    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            /*pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading*/

            
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(40);
        }
    });


   

    pros::Task isStuckTask(
        is_stuck_check,
        nullptr,
        "stuck task"
    );

    pros::Task ColorTask(
        color_check,
        nullptr,
        "color check task"
    );

    pros::Task clampTask(
        clampCheck,
        nullptr,
        "clamp task"
    );

}

void disabled() {}

void competition_initialize() {}

// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(example2_txt); // '.' replaced with "_" to make c++ happy

int autonRoute = 3;
void autonomous() {
    selector.run_auton();
    std::cout << "Running best auton" << std::endl;
    //blue SAWP
    if (autonRoute == 0){
        char team_color = 'B';
        auton = true;
        clampOn = true;
        chassis.moveToPoint(0, -25, 1000, {.forwards = false, .maxSpeed = 75, .minSpeed = 30, .earlyExitRange=8});
        chassis.moveToPoint(0, -43, 1000, {.forwards = false, .maxSpeed = 40});
        while(!current){
            pros::delay(10);
        }
        pros::delay(100);
        intake_on = true;
        chassis.turnToHeading(-60, 1000);
        chassis.waitUntilDone();
        pros::delay(100);
        chassis.setPose(0, 0, 0);
        pros::delay(200);
        intake_on = false;
        ringStop = true;
        chassis.moveToPoint(0, 30, 1000);
        while(!is_ring_stopped){
            pros::delay(50);
        }
        chassis.turnToHeading(-40, 1000);
        chassis.moveToPoint(28, -30, 2000, {.forwards = false, .maxSpeed = 80/*, .minSpeed = 40, .earlyExitRange = 8*/});
        
        /*chassis.moveToPose(30, -35, -32, 1000, {.forwards = false});*/
        chassis.waitUntilDone();
        pros::delay(100);
        clampOn = false;
        pros::delay(500);
        //chassis.moveToPoint(29, -32.5, 1000, {.forwards = false, .maxSpeed = 50});
        chassis.turnToHeading(55, 1000);
        chassis.waitUntilDone();
        clampOn = true;
        pros::delay(150);
        chassis.setPose(0, 0, 0);
        chassis.moveToPoint(0, -30, 2000, {.forwards = false, .maxSpeed = 40});
        
        while(!current){
            pros::delay(10);
        }
        pros::delay(100);
        ringStop = false;
        intake_on = true;
        chassis.turnToHeading(105, 1000);
        chassis.waitUntilDone();
        pros::delay(100);
        chassis.setPose(0, 0, 0);
        chassis.moveToPoint(0, 20, 1000);
        pros::delay(250);
        chassis.moveToPoint(0, 16, 1000);
        chassis.turnToHeading(-165, 1000, {.direction = AngularDirection::CCW_COUNTERCLOCKWISE});
        chassis.waitUntilDone();
        pros::delay(100);
        chassis.setPose(0, 0, 0);
        chassis.moveToPoint(0, 25, 1000, {.maxSpeed = 90, .minSpeed = 60, .earlyExitRange = 4});
        
        
    }
    if (autonRoute == 1){
        char team_color = 'B';
        //  2/3 blue AWP
        auton = true;
        clampOn = true;
        chassis.moveToPoint(0, -25, 1000, {.forwards = false, .maxSpeed = 75, .minSpeed = 30, .earlyExitRange=8});
        chassis.moveToPoint(0, -40, 1000, {.forwards = false, .maxSpeed = 40});
        while(!current){
            pros::delay(10);
        }
        pros::delay(100);
        intake_on = true;
        pros::delay(500);
        chassis.moveToPoint(0, -30, 1000, {.maxSpeed = 60});
        chassis.turnToHeading(-121, 1000);
        chassis.moveToPoint(-23, -57, 2000, {.maxSpeed = 60});
        chassis.waitUntilDone();
        pros::delay(500);
        chassis.moveToPoint(-11, -47, 1000, {.forwards = false});
        chassis.turnToHeading(-70, 1000);
        chassis.moveToPoint(-24, -42, 1000);
        chassis.waitUntilDone();
        pros::delay(250);
        chassis.moveToPoint(-19, -43, 1000, {.forwards = false, .minSpeed = 30, .earlyExitRange = 5});
        chassis.turnToHeading(-250, 1000);
        chassis.moveToPoint(34, -59, 1000, {.maxSpeed = 60, .minSpeed = 20, .earlyExitRange = 11});
        
        chassis.waitUntilDone();
        pros::delay(1000);
        intake_on = false;
        
        //chassis.moveToPoint(0, 0, 1000, {.forwards = false});
    }
    else if(autonRoute == 2){
        //red SAWP
        char team_color = 'R';
        auton = true;
        clampOn = true;
        chassis.moveToPoint(0, -25, 1000, {.forwards = false, .maxSpeed = 75, .minSpeed = 30, .earlyExitRange=8});
        chassis.moveToPoint(0, -40, 1000, {.forwards = false, .maxSpeed = 40});
        while(!current){
            pros::delay(10);
        }
        pros::delay(100);
        intake_on = true;
        chassis.turnToHeading(60, 1000);
        chassis.waitUntilDone();
        pros::delay(100);
        chassis.setPose(0, 0, 0);
        pros::delay(200);
        intake_on = false;
        ringStop = true;
        chassis.moveToPoint(0, 30, 1000);
        while(!is_ring_stopped){
            pros::delay(50);
        }
        chassis.turnToHeading(25, 1000);
        chassis.moveToPoint(-16, -8, 3000, {.forwards = false, .maxSpeed = 100, .minSpeed = 60, .earlyExitRange = 5});
        chassis.moveToPoint(-27, -33, 1000, {.forwards = false});
        chassis.waitUntilDone();
        pros::delay(100);
        clampOn = false;
        pros::delay(500);
        //chassis.moveToPoint(29, -32.5, 1000, {.forwards = false, .maxSpeed = 50});
        chassis.turnToHeading(-48, 1000);
        chassis.waitUntilDone();
        clampOn = true;
        pros::delay(150);
        chassis.setPose(0, 0, 0);
        chassis.moveToPoint(0, -30, 2000, {.forwards = false, .maxSpeed = 40});
        while(!current){
            pros::delay(10);
        }
        pros::delay(100);
        ringStop = false;
        intake_on = true;
        chassis.turnToHeading(-100, 1000);
        chassis.waitUntilDone();
        pros::delay(100);
        chassis.setPose(0, 0, 0);
        chassis.moveToPoint(0, 20, 1000);
        pros::delay(250);
        chassis.moveToPoint(0, 16, 1000);
        chassis.turnToHeading(155, 1000, {.direction = AngularDirection::CW_CLOCKWISE});
        chassis.waitUntilDone();
        pros::delay(100);
        chassis.setPose(0, 0, 0);
        chassis.moveToPoint(0, 35, 1000, {.minSpeed = 30, .earlyExitRange = 10});

        /*chassis.moveToPoint(0, 16, 1000);
        chassis.turnToHeading(165, 1000, {.direction = AngularDirection::CW_CLOCKWISE});
        chassis.waitUntilDone();
        pros::delay(100);
        chassis.setPose(0, 0, 0);
        chassis.moveToPoint(0, 25, 1000, {.maxSpeed = 90, .minSpeed = 60, .earlyExitRange = 4});*/
        
    }
    else if(autonRoute == 3){
        //blue rush
        char team_color = 'B';
        chassis.moveToPoint(0, 33, 1000, {.minSpeed = 110});
        chassis.waitUntilDone();
        doinker.set_value(true);
        chassis.moveToPoint(0, -10, 1000, {.forwards = false, .maxSpeed = 60, .earlyExitRange = 4});
        //chassis.moveToPoint(0, 0, 1000, {.forwards = false});
        chassis.waitUntilDone();
        doinker.set_value(false);
        pros::delay(500);
        chassis.turnToHeading(-170, 1000, {.maxSpeed = 60});
        chassis.waitUntilDone();
        pros::delay(200);
        chassis.setPose(0, 0, 0);
        chassis.moveToPoint(0, -20, 1000, {.forwards = false, .maxSpeed = 40});
        clampOn = true;
        while(!current){
            pros::delay(10);
        }
        chassis.cancelAllMotions();
        pros::delay(100);
        intake_on = true;
        pros::delay(400);
        chassis.moveToPoint(0, 15, 3000, {.maxSpeed = 70, .minSpeed = 20, .earlyExitRange = 5});
        chassis.turnToHeading(-140, 5000, {.maxSpeed = 60});
        chassis.waitUntilDone();
        pros::delay(100);
        clampOn = false;
        intake_on = false;
        chassis.setPose(0, 0, 0);
        chassis.moveToPoint(0, 12, 1000);
        chassis.turnToHeading(-180, 1000);
        chassis.waitUntilDone();
        clampOn = true;
        chassis.moveToPoint(0, 30, 1000, {.forwards = false, .maxSpeed = 40});
        while(!current){
            pros::delay(10);
        }
        chassis.cancelAllMotions();
        chassis.waitUntilDone();
        chassis.turnToHeading(50, 1000);

    }
    else if (autonRoute == 4){
        //red 2/3
        char team_color = 'R';
        auton = true;
        clampOn = true;
        chassis.moveToPoint(0, -25, 1000, {.forwards = false, .maxSpeed = 75, .minSpeed = 30, .earlyExitRange=8});
        chassis.moveToPoint(0, -40, 1000, {.forwards = false, .maxSpeed = 40});
        while(!current){
            pros::delay(10);
        }
        pros::delay(100);
        intake_on = true;
        pros::delay(500);
        chassis.moveToPoint(0, -30, 1000, {.maxSpeed = 60});
        chassis.turnToHeading(121, 1000);
        chassis.moveToPoint(21, -41, 2000, {.maxSpeed = 50});
        chassis.waitUntilDone();
        pros::delay(1000);
        chassis.moveToPoint(5, -30, 2000, {.forwards = false ,.maxSpeed = 50});
        chassis.turnToHeading(75, 1000);
        chassis.moveToPoint(16, -19, 1000);
        pros::delay(1000);
        chassis.moveToPoint(11, -21, 1000, {.forwards = false, .earlyExitRange = 4});
        chassis.turnToHeading(-115, 1000);
        chassis.moveToPoint(-23, -41,  10000, {.maxSpeed = 50, .minSpeed = 30, .earlyExitRange = 7});


        chassis.waitUntilDone();
        intake_on = false;
        
    }
    else if (autonRoute == 5){
        //red rush
        chassis.moveToPoint(0, 30, 1000, {.minSpeed = 110});
        chassis.waitUntilDone();
        pros::delay(100);
        doinker.set_value(true);
        chassis.moveToPoint(0, -30, 1000, {.forwards = false, .maxSpeed = 60, .earlyExitRange = 4});
        chassis.waitUntilDone();
        doinker.set_value(false);
        pros::delay(500);
        chassis.turnToHeading(-170, 1000, {.maxSpeed = 60});
        chassis.waitUntilDone();
        pros::delay(200);
        chassis.setPose(0, 0, 0);
        chassis.moveToPoint(0, -20, 1000, {.forwards = false, .maxSpeed = 40});
        clampOn = true;
        while(!current){
            pros::delay(10);
        }
        chassis.cancelAllMotions();
        pros::delay(100);
        intake_on = true;
        chassis.moveToPoint(0, 20, 3000, {.maxSpeed = 70, .minSpeed = 20, .earlyExitRange = 5});
        chassis.turnToHeading(140, 5000, {.maxSpeed = 60});
        chassis.waitUntilDone();
        pros::delay(100);
        clampOn = false;
        intake_on = false;
        chassis.setPose(0, 0, 0);
        chassis.moveToPoint(0, 12, 1000);
        chassis.turnToHeading(-170, 1000);
        chassis.waitUntilDone();
        clampOn = true;
        chassis.moveToPoint(0, 30, 1000, {.forwards = false, .maxSpeed = 40});
        while(!current){
            pros::delay(10);
        }
        chassis.cancelAllMotions();
    }

}


/**
 * Runs in driver control
 */
void opcontrol() {

    pros::Task lbPidTask(
        LBpidTask,
        nullptr,
        "LB PID Task"
    );

    opC = true;
    auton = false;

    bool clampState = true;
    bool doinkerState = false;


    ringStop = false;

    while (true) {
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        chassis.arcade(leftY, rightX, false, 0.35);


        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
            clampState = !clampState; // Toggle piston state
            clamp.set_value(clampState);
        } 

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
            doinkerState = !doinkerState; // Toggle piston state
            doinker.set_value(doinkerState);
        } 

        // Intake control
        
            if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
                //intake_on = true;
                if(!is_stuck){
                    reversed = false;
                    intake1.move(-127);
                }
                
            } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
                reversed = true;
                intake1.move(127);
            }
            
            else {
                reversed = false;
                intake1.move(0);
            }
        

        pros::delay(15);  // Small delay to avoid overwhelming the system
}
}


