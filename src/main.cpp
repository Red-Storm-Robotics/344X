#include "main.h"

Controller controller;
MotorGroup leftDrive = MotorGroup({-1, -9});
MotorGroup rightDrive = MotorGroup({2, 10});
auto drive = ChassisControllerFactory::create(leftDrive, rightDrive, AbstractMotor::gearset::green, {4.1_in, 11.5_in});

MotorGroup intake = MotorGroup({3, -8});

Motor trayTilter = Motor(5);

bool tankDrive = true;
bool enableAuton = true;

void toggleDriveMode() {
  	tankDrive = !tankDrive;
	pros::lcd::clear_line(1);
	if (tankDrive) {
	    pros::lcd::set_text(1, "Tank Drive Mode");
  	} else {
	   	pros::lcd::set_text(1, "Arcade Drive Mode");
	}
}

void toggleAuton() {
  	enableAuton = !enableAuton;
	pros::lcd::clear_line(2);
	if (enableAuton) {
	    pros::lcd::set_text(2, "Autonomous ON");
	} else {
	    pros::lcd::set_text(2, "Autonomous OFF");
	}
}

ControllerButton intakeFwd = ControllerButton(ControllerDigital::L1);
ControllerButton intakeRev = ControllerButton(ControllerDigital::L2);

#define INTAKE_TOGGLE true

int intakeMode = 0;

ControllerButton trayStowed = ControllerButton(ControllerDigital::A);
ControllerButton trayIntaking = ControllerButton(ControllerDigital::B);
ControllerButton trayStacking = ControllerButton(ControllerDigital::X);
ControllerButton trayTowering = ControllerButton(ControllerDigital::Y);

ControllerButton trayManualUp = ControllerButton(ControllerDigital::R1);
ControllerButton trayManualDown = ControllerButton(ControllerDigital::R2);

bool trayManualMode = false;

ControllerButton runDropRoutine = ControllerButton(ControllerDigital::right);

/**
* Runs initialization code. This occurs as soon as the program is started.
*
* All other competition modes are blocked by initialize; it is recommended
* to keep execution time for this mode under a few seconds.
*/
void initialize() {
	pros::delay(10);
    pros::lcd::initialize();
	pros::lcd::set_text(0, "344X York Meet Program");
	pros::lcd::set_text(1, "Tank Drive Mode");
	pros::lcd::set_text(2, "Autonomous ON");

	pros::lcd::set_text(4, "Press First Button to Toggle Drive Mode");
	pros::lcd::set_text(5, "Press Second Button to Toggle Auton");
	pros::lcd::set_text(6, "Ports: Drive (LF, RF, LR, RR) => (1, 2, 9, 10)");
	pros::lcd::set_text(7, "Intake (L, R) => (3, 8) and Tilter 5");

	pros::lcd::register_btn0_cb(toggleDriveMode);
    pros::lcd::register_btn1_cb(toggleAuton);
}

/**
* Runs while the robot is in the disabled state of Field Management System or
* the VEX Competition Switch, following either autonomous or opcontrol. When
* the robot is enabled, this task will exit.
*/
void disabled() {}

/**
* Runs after initialize(), and before autonomous when connected to the Field
* Management System or the VEX Competition Switch. This is intended for
* competition-specific initialization routines, such as an autonomous selector
* on the LCD.
*
* This task will exit when the robot is enabled and autonomous or opcontrol
* starts.
*/
void competition_initialize() {}

/**
* Runs the user autonomous code. This function will be started in its own task
* with the default priority and stack size whenever the robot is enabled via
* the Field Management System or the VEX Competition Switch in the autonomous
* mode. Alternatively, this function may be called in initialize or opcontrol
* for non-competition testing purposes.
*
* If the robot is disabled or communications is lost, the autonomous task
* will be stopped. Re-enabling the robot will restart the task, not re-start it
* from where it left off.
*/
void autonomous() {
    drive.moveDistance(24_in);
    drive.waitUntilSettled();
    drive.moveDistance(-24_in);
    drive.waitUntilSettled();
}

/**
* Runs the operator control code. This function will be started in its own task
* with the default priority and stack size whenever the robot is enabled via
* the Field Management System or the VEX Competition Switch in the operator
* control mode.
*
* If no competition control is connected, this function will run immediately
* following initialize().
*
* If the robot is disabled or communications is lost, the
* operator control task will be stopped. Re-enabling the robot will restart the
* task, not resume it from where it left off.
*/
void opcontrol() {
	while (true) {

        // DRIVETRAIN
		if (tankDrive) {
			float left = controller.getAnalog(ControllerAnalog::leftY);
			float right = controller.getAnalog(ControllerAnalog::rightY);
			// Cubic Curving for better manuverability
			left = left*left*left;
			right = right*right*right;
			drive.tank(left, right);
		} else { // Arcade Drive
			float speed = controller.getAnalog(ControllerAnalog::leftY);
			float yaw = controller.getAnalog(ControllerAnalog::rightX);
			// Cubic Curving for better manuverability
			speed = speed*speed*speed;
			yaw = yaw*yaw*yaw;
			drive.arcade(speed, yaw);
		}

        // INTAKE

#if INTAKE_TOGGLE
        if (intakeMode == -1) {
            if (intakeFwd.changedToPressed()) {
                intakeMode = 1;
            } else if (intakeRev.changedToPressed()) {
                intakeMode = 0;
            }
        } else if (intakeMode == 0) {
            if (intakeFwd.changedToPressed()) {
                intakeMode = 1;
            } else if (intakeRev.changedToPressed()) {
                intakeMode = -1;
            }
        } else if (intakeMode == 1) {
            if (intakeRev.changedToPressed()) {
                intakeMode = -1;
            } else if (intakeFwd.changedToPressed()) {
                intakeMode = 0;
            }
        }
#else
        if (intakeFwd.isPressed()) {
            intakeMode = 1;
        } else if (intakeRev.isPressed()) {
            intakeMode = -1;
        } else {
            intakeMode = 0;
        }
#endif

        if (intakeMode == -1) {
            intake.moveVelocity(-50);
        } else if (intakeMode == 1) {
            intake.moveVelocity(100);
        } else {
            intake.moveVelocity(0);
        }


        // TRAY TILTER

        if (trayStowed.changedToPressed()) {
            trayTilter.moveAbsolute(0, 50);
            trayManualMode = false;
        } else if (trayIntaking.changedToPressed()) {
            trayTilter.moveAbsolute(30, 50);
            trayManualMode = false;
        } else if (trayTowering.changedToPressed()) {
            trayTilter.moveAbsolute(60, 50);
            trayManualMode = false;
        } else if (trayStacking.changedToPressed()) {
            trayTilter.moveAbsolute(90, 50);
            trayManualMode = false;
        } else if (trayManualUp.isPressed()) {
            trayTilter.moveVelocity(50);
            trayManualMode = true;
        } else if (trayManualDown.isPressed()) {
            trayTilter.moveVelocity(-50);
            trayManualMode = true;
        } else if (trayManualMode) {
            trayTilter.moveVelocity(0);
        }

        // AUTO DROP

        if (runDropRoutine.changedToPressed()) {
            drive.stop();

            trayTilter.moveAbsolute(90, 50);
            while (abs(trayTilter.getPosition() - 90) > 5) {
                pros::delay(20);
            }

            pros::delay(500);

            trayTilter.moveAbsolute(30, 50);
            intake.moveVelocity(-50);

            while (abs(trayTilter.getPosition() - 90) > 5) {
                pros::delay(20);
            }

            intake.moveVelocity(0);

            drive.moveDistance(-6_in);
            drive.waitUntilSettled();

        }

        pros::delay(20);
	}
}
