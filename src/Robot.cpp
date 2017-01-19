#include <iostream>
#include <memory>
#include <string>
#include <CameraServer.h>
#include <Joystick.h>
#include <CANTalon.h>
#include <RobotDrive.h>
#include "AHRS.h"
#include <math.h>
#include "WPILib.h"
#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
class Robot: public frc::IterativeRobot {
CANTalon* leftFront;
CANTalon* leftRear;
CANTalon* rightFront;
CANTalon* rightRear;
RobotDrive *myRobot;
Joystick* stick;
Joystick* stick2;
AHRS* navX;
bool joystick1_1;
bool joystick2_1;
bool joystick3_1;
bool joystick4_1;
bool joystick5_1;
bool joystick6_1;
bool joystick7_1;
bool joystick8_1;
bool joystick1_2;
bool joystick2_2;
bool joystick3_2;
bool joystick4_2;
bool joystick5_2;
bool joystick6_2;
bool joystick7_2;
bool joystick8_2;
double joystickY_1;
double joystickY_2;
double joystickX_1;
double joystickX_2;
bool slowBool;
bool interlock;
double joyLeftValue;
double joyRightValue;
double encodeDistance;
double autoGoDistance;
double encoderCompensation;
Encoder *leftEncode;
Encoder *rightEncode;
CameraServer* camera;
public:
	void RobotInit() {
		chooser.AddDefault(autoNameDefault, autoNameDefault);
		chooser.AddObject(autoNameCustom, autoNameCustom);
		frc::SmartDashboard::PutData("Auto Modes", &chooser);
		leftFront = new CANTalon { 2 };
		leftRear =  new CANTalon { 3 };
		rightFront = new CANTalon { 4 };
		rightRear = new CANTalon { 5 };
		myRobot = new RobotDrive {leftFront, leftRear, rightFront, rightRear};
		stick = new Joystick { 0 };
		stick2 = new Joystick { 1 };
		navX = new AHRS { SPI::Port::kMXP };
		navX->Reset();
		leftEncode = new Encoder { 2, 3, false, Encoder::EncodingType::k4X};
		rightEncode = new Encoder { 0, 1, false, Encoder::EncodingType::k4X};
		encoderCompensation = 0.94224842556;
		camera->GetInstance();
		camera->StartAutomaticCapture();
	}

	/*
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * GetString line to get the auto name from the text box below the Gyro.
	 *
	 * You can add additional auto modes by adding additional comparisons to the
	 * if-else structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	void AutonomousInit() override {
		autoSelected = chooser.GetSelected();
		// std::string autoSelected = SmartDashboard::GetString("Auto Selector", autoNameDefault);
		std::cout << "Auto selected: " << autoSelected << std::endl;

		if (autoSelected == autoNameCustom) {
			// Custom Auto goes here
		} else {
			leftEncode->Reset();
			rightEncode->Reset();
			encodeDistance = 7.4 * M_PI / 270;
			leftEncode->SetDistancePerPulse(encodeDistance);
			rightEncode->SetDistancePerPulse(encodeDistance);
			leftEncode->SetMinRate(.5);
			rightEncode->SetMinRate(.5);
			/* while (abs(rightEncode->GetDistance()) <= 27) {
				myRobot->TankDrive(-1, -1);
			} */
			while (abs(navX->GetAngle()) < 165) {
				SmartDashboard::PutNumber("Angle",navX->GetAngle());
				myRobot->TankDrive(.6,-.6);
			}
		}
	}

	void AutonomousPeriodic() {
		if (autoSelected == autoNameCustom) {
			// Custom Auto goes here
		} else {
			// Default Auto goes here
		}
	}

	void TeleopInit() {
		interlock = true;
		slowBool = false;
	}

	void TeleopPeriodic() {
		joystick1_1 = stick->GetRawButton(1);
		joystick2_1 = stick->GetRawButton(2);
		joystick3_1 = stick->GetRawButton(3);
		joystick4_1 = stick->GetRawButton(4);
		joystick5_1 = stick->GetRawButton(5);
		joystick6_1 = stick->GetRawButton(6);
		joystick7_1 = stick->GetRawButton(7);
		joystick8_1 = stick->GetRawButton(8);
		joystick1_2 = stick2->GetRawButton(1);
		joystick2_2 = stick2->GetRawButton(2);
		joystick3_2 = stick2->GetRawButton(3);
		joystick4_2 = stick2->GetRawButton(4);
		joystick5_2 = stick2->GetRawButton(5);
		joystick6_2 = stick2->GetRawButton(6);
		joystick7_2 = stick2->GetRawButton(7);
		joystick8_2 = stick2->GetRawButton(8);
		joystickY_1 = stick->GetRawAxis(1);
		joystickY_2 = stick2->GetRawAxis(1);
		joystickX_1 = stick->GetRawAxis(0);
		joystickX_2 = stick2->GetRawAxis(0);
		if (joystick3_2 && interlock) {
			slowBool = true;
			interlock = false;
		}
		if (!joystick3_2) {
			interlock = true;
		}
		if (slowBool) {
			joyLeftValue = joystickY_1 * .5;
			joyRightValue = joystickY_2 * .5;
		} else {
			joyLeftValue = joystickY_1;
			joyRightValue = joystickY_2;
		}
		myRobot->TankDrive(joyLeftValue, joyRightValue, true);
	}

	void TestPeriodic() {
		lw->Run();
	}

private:
	frc::LiveWindow* lw = LiveWindow::GetInstance();
	frc::SendableChooser<std::string> chooser;
	const std::string autoNameDefault = "Default";
	const std::string autoNameCustom = "My Auto";
	std::string autoSelected;
};

START_ROBOT_CLASS(Robot)
