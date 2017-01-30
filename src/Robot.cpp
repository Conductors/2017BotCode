#include <iostream>
#include <memory>
#include <string>
#include <CameraServer.h>
#include <Joystick.h>
#include <CANTalon.h>
#include <RobotDrive.h>
#include <Timer.h>
#include "AHRS.h"
#include <math.h>
#include "WPILib.h"
#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <DigitalInput.h>
#include <I2C.h>
class Robot: public frc::IterativeRobot {
CANTalon* leftTop;
CANTalon* leftBottom;
CANTalon* rightTop;
CANTalon* rightBottom;
RobotDrive *myRobot;
Joystick* stick;
Joystick* stick2;
AHRS* navX;
Timer timer1;
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
SerialPort* arduino;
DigitalInput* limitOne;
bool limitInterlock;
Compressor *mainCompress;
I2C *rioduino;
public:
	void RobotInit() {
		chooser.AddDefault(autoMiddle, autoMiddle);
		chooser.AddObject(autoLeft,autoLeft);
		frc::SmartDashboard::PutData("Auto Modes", &chooser);
		//Sets Talon SRX CAN IDs
		leftTop = new CANTalon { 1 };
		leftBottom =  new CANTalon { 0 };
		rightTop = new CANTalon { 2 };
		rightBottom = new CANTalon { 3 };
		//Initiates robot with the 4 Talons
		myRobot = new RobotDrive { leftTop, leftBottom, rightTop, rightBottom };
		//Initiates joysticks
		stick = new Joystick { 0 };
		stick2 = new Joystick { 1 };
		//Adds NavX (The large one)
		navX = new AHRS { SPI::Port::kMXP };
		navX->Reset();
		leftEncode = new Encoder { 2, 3, false, Encoder::EncodingType::k4X };
		rightEncode = new Encoder { 0, 1, false, Encoder::EncodingType::k4X };
		//Encoder compensation to balance out the two sides (One will always go faster)
		//arduino = new SerialPort(9600,SerialPort::Port::kUSB,8,SerialPort::kParity_None,SerialPort::kStopBits_One);
		//CameraServer::GetInstance()->StartAutomaticCapture();
		encodeDistance = SmartDashboard::PutNumber("1st distance", 111);
		limitOne = new DigitalInput (6);
		mainCompress = new Compressor (0);
		mainCompress->SetClosedLoopControl(true);
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
		limitInterlock = true;
		//Resets all encoder values (so that if teleop was used previously
		//it won't mess with distance
		leftEncode->Reset();
		rightEncode->Reset();
		//Calculates distance per pulse using wheel diameter * Pi / pulse count (270)
		encodeDistance = 7.4 * M_PI / 270;
		leftEncode->SetDistancePerPulse(encodeDistance);
		rightEncode->SetDistancePerPulse(encodeDistance);
		//Sets minimum wheel rotation rate before it's considered stopped
		leftEncode->SetMinRate(.5);
		rightEncode->SetMinRate(.5);
		//Robot drives number of inches forward (remember to subtract length of robot)
		encodeDistance = SmartDashboard::GetNumber("1st distance", 111);
		while (abs(rightEncode->GetDistance()) <= 27) {
			myRobot->TankDrive(-.6, -.6);
			if (limitOne->Get() == true) {
				limitInterlock = true;
			}
			if (limitOne->Get() == false) {
				leftEncode->Reset();
				leftEncode->SetDistancePerPulse(encodeDistance);
				limitInterlock = false;
				while (abs(leftEncode->GetDistance())  < 10) {
					myRobot->TankDrive(.6,.6);
				}
				Wait(3);
		}
		Wait(2);
		//Rotates bot until it's 180 degrees. About 15 degrees per loop, so subtract that.
		navX->Reset();
		while (abs(navX->GetAngle()) < 20) {
			SmartDashboard::PutNumber("Angle",navX->GetAngle());
			myRobot->TankDrive(.6,-.6);
		}
		}
	}

	void AutonomousPeriodic() {
	}
	void TeleopInit() {
		interlock = true;
		slowBool = false;
		limitInterlock = true;
	}

	void TeleopPeriodic() {
		//Updates all joystick values
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
		//Need interlock to avoid rapid button switching
		if (joystick3_2 && interlock) {
			slowBool = !slowBool;
			interlock = false;
		}
		if (!joystick3_2) {
			interlock = true;
		}
		if (slowBool) {
			joyLeftValue = joystickY_1 * .6;
			joyRightValue = joystickY_2 * .6;
		} else {
			joyLeftValue =  joystickY_1;
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
	const std::string autoLeft = "Left";
	const std::string autoMiddle = "Middle";
	const std::string autoRight = "Right";
	std::string autoSelected;
};

START_ROBOT_CLASS(Robot)
