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
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <thread>
class Robot: public frc::IterativeRobot {
CANTalon* leftTop;
CANTalon* leftBottom;
CANTalon* rightTop;
CANTalon* rightBottom;
CANTalon* intake;
CANTalon* shooter;
CANTalon* climber;
CANTalon* agitator;
CANTalon* unassigned;
RobotDrive *myRobot;
Joystick* stick;
Joystick* stick2;
Joystick* stick3;
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
bool joystick1_1_Inter;
bool joystick2_1_Inter;
bool joystick3_1_Inter;
bool joystick4_1_Inter;
bool joystick5_1_Inter;
bool joystick6_1_Inter;
bool joystick7_1_Inter;
bool joystick8_1_Inter;
bool joystick1_2_Inter;
bool joystick2_2_Inter;
bool joystick3_2_Inter;
bool joystick4_2_Inter;
bool joystick5_2_Inter;
bool joystick6_2_Inter;
bool joystick7_2_Inter;
bool joystick8_2_Inter;
bool joystick1_3;
bool joystick2_3;
bool joystick3_3;
bool joystick4_3;
bool joystick5_3;
bool joystick6_3;
bool joystick7_3;
bool joystick8_3;
bool joystick1_3_Inter;
bool joystick2_3_Inter;
bool joystick3_3_Inter;
bool joystick4_3_Inter;
bool joystick5_3_Inter;
bool joystick6_3_Inter;
bool joystick7_3_Inter;
bool joystick8_3_Inter;
double joystickRT_3;
double joystickLT_3;
double joystickY_1;
double joystickY_2;
double joystickX_1;
double joystickX_2;
bool directBool;
double joyLeftValue;
double joyRightValue;
double encodeDistance;
double autoGoDistance;
double encoderCompensation;
double initialAngle;
double currentAngle;
double tolerance;
bool ejectBool;
Encoder *leftEncode;
Encoder *rightEncode;
cs::UsbCamera camera;
cs::UsbCamera camera2;
SerialPort* arduino;
DigitalInput* limitOne;
bool limitInterlock;
Compressor *mainCompress;
std::vector<double> cenX;
//Defines rioduino I2C connection for later use
I2C *rioduino;
DoubleSolenoid* shifting;
DoubleSolenoid* gear;
DoubleSolenoid* gearEject;
Talon* leftTal;
Talon* rightTal;
int autoCounter;
bool shootBool;
public:
	std::shared_ptr<NetworkTable> table1;
	void RobotInit() {
		chooser.AddDefault(autoMiddle, autoMiddle);
		chooser.AddObject(autoLeft,autoLeft);
		chooser.AddObject(autoRight,autoRight);
		chooser.AddObject(shootRight,shootRight);
		chooser.AddObject(shootLeft,shootLeft);
		chooser.AddObject(gearPassive,gearPassive);
		chooser.AddObject(autoTest,autoTest);
		frc::SmartDashboard::PutData("Auto Modes", &chooser);
		//Sets Talon SRX CAN IDs
		leftTop = new CANTalon { 6 };
		leftBottom =  new CANTalon { 1 };
		rightTop = new CANTalon { 2 };
		rightBottom = new CANTalon { 5 };
		intake = new CANTalon { 3 };
		shooter = new CANTalon { 4 };
		climber = new CANTalon { 7 };
		//Initiates robot with the 4 Talons
		myRobot = new RobotDrive { leftTop, leftBottom, rightTop, rightBottom };
		//Initiates joysticks
		stick = new Joystick { 1 };
		stick2 = new Joystick { 0 };
		unassigned = new CANTalon { 11 };
		agitator = new CANTalon { 10 };
		//Adds NavX (The large one)
		navX = new AHRS { SPI::Port::kMXP };
		navX->Reset();
		//Adds encoders
		rightEncode = new Encoder { 0, 1, false, Encoder::EncodingType::k4X };
		leftEncode = new Encoder { 2, 3, false, Encoder::EncodingType::k4X };
		//Encoder compensation to balance out the two sides (One will always go faster)
		//arduino = new SerialPort(9600,SerialPort::Port::kUSB,8,SerialPort::kParity_None,SerialPort::kStopBits_One);
		encodeDistance = SmartDashboard::PutNumber("1st distance", 111);
		limitOne = new DigitalInput (6);
		//Defining pneumatics things
		mainCompress = new Compressor { 0 };
		mainCompress->SetClosedLoopControl(true);
		shifting = new DoubleSolenoid(0,1);
		gear = new DoubleSolenoid(2,3);
		gearEject = new DoubleSolenoid(4,5);
		//Creates network table for GRIP data
		table1 = NetworkTable::GetTable("GRIP/myContoursReport");
		//Initializes camera server
		CameraServer::GetInstance()->StartAutomaticCapture();
		stick3 = new Joystick (2);
		gear->Set(DoubleSolenoid::kReverse);
		shifting->Set(DoubleSolenoid::kReverse);
		gearEject->Set(DoubleSolenoid::kReverse);
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
		//std::string autoSelected = SmartDashboard::GetString("Auto Selector", autoMiddle);
		std::cout << "Auto selected: " << autoSelected << std::endl;
		limitInterlock = true;
		//Resets all encoder values (so that if teleop was used previously
		leftEncode->Reset();
		rightEncode->Reset();
		encodeDistance = 6 * M_PI / 270;
		leftEncode->SetDistancePerPulse(encodeDistance);
		rightEncode->SetDistancePerPulse(encodeDistance);
		//Sets minimum wheel rotation rate before it's considered stopped
		leftEncode->SetMinRate(.5);
		rightEncode->SetMinRate(.5);
		myRobot->SetExpiration(1000000);
		//Robot drives number of inches forward (remember to subtract length of robot)
		encodeDistance = SmartDashboard::GetNumber("1st distance", 111);
		cenX = table1->GetNumberArray("centerX",llvm::ArrayRef<double>());
		//Makes counter for different steps of auto
		autoCounter = 0;
		shifting->Set(shifting->kReverse);
	}

	void AutonomousPeriodic() {
			 /* switch(autoCounter) {
				case 0: {
					if (abs(rightEncode->GetDistance()) > 68) {
						autoCounter = 1;
						rightEncode->Reset();
						rightEncode->SetDistancePerPulse(encodeDistance);
						break;
					} else {
						myRobot->TankDrive(.6,.6);
						SmartDashboard::PutNumber("Encode 2",rightEncode->GetDistance());
						break;
					}
				}
				case 1:
					myRobot->TankDrive(0.0,0.0);
					break;
			} */
			 if(autoSelected == autoRight) {
				switch(autoCounter) {
							case 0: {
								SmartDashboard::PutNumber("Encoder: ", abs(leftEncode->GetDistance()));
								if (abs(leftEncode->GetDistance()) > 27) {
									myRobot->TankDrive(.0,.0);
									autoCounter++;
									leftEncode->Reset();
									leftEncode->SetDistancePerPulse(encodeDistance);
									navX->Reset();
									break;
								} else {
									myRobot->TankDrive(.6,.6);
									break;
								}
							}
							/* case 1: {
								cenX = table1->GetNumberArray("centerX",llvm::ArrayRef<double>());
								SmartDashboard::PutNumber("Angle: ", navX->GetAngle());
								double center = abs(cenX[1] - cenX[0]) + cenX[0];
								if (center > 300 && center < 400) {
									autoCounter++;
									break;
								} else {
									if (center > 100) {
										myRobot->TankDrive(.4,-.4);
									}
									else if (center < 540) {
										myRobot->TankDrive(.4,-.4);
									}
									break;
								}
							} */
							case 1: {
								SmartDashboard::PutNumber("Angle: ", navX->GetAngle());
								if(abs(navX->GetAngle()) > 45) {
									myRobot->TankDrive(.0,.0);
									navX->Reset();
									leftEncode->Reset();
									autoCounter++;
									break;
								} else {
									myRobot->TankDrive(-.4,.4);
									break;
								}
							}
							case 2: {
								std::cout << "case2";
								if (abs(leftEncode->GetDistance()) > 15) {
									myRobot->TankDrive(.0,.0);
									autoCounter++;
									break;
								} else {
									myRobot->TankDrive(.6,.6);
									break;
								}
							}
							case 3: break;
						}
			}
			else if (autoSelected == autoMiddle) {
				switch(autoCounter) {
							case 0: {
								SmartDashboard::PutNumber("Encoder: ", abs(leftEncode->GetDistance()));
								if (abs(leftEncode->GetDistance()) > 33) {
									autoCounter++;
									leftEncode->Reset();
									myRobot->TankDrive(.0,.0);
									break;
								} else {
									myRobot->TankDrive(.6,.6);
									break;
								}
							}
							case 1: {
								SmartDashboard::PutNumber("Gear",1);
								gear->Set(DoubleSolenoid::kForward);
								Wait(1);
								gearEject->Set(DoubleSolenoid::kForward);
								autoCounter++;
								break;
							}
							case 2: {
								if (abs(leftEncode->GetDistance()) > 12) {
									autoCounter++;
									leftEncode->Reset();
									leftEncode->SetDistancePerPulse(encodeDistance);
									myRobot->TankDrive(.0,.0);
									break;
								} else {
									myRobot->TankDrive(-.6,-.6);
									break;
								}
							}
							case 3: break;
						}
			}
			else if (autoSelected == gearPassive) {
							switch(autoCounter) {
										case 0: {
											SmartDashboard::PutNumber("Encoder: ", abs(leftEncode->GetDistance()));
											if (abs(leftEncode->GetDistance()) > 33) {
												autoCounter++;
												leftEncode->Reset();
												myRobot->TankDrive(.0,.0);
												break;
											} else {
												myRobot->TankDrive(.6,.6);
												break;
											}
										}
										case 1: break;
									}
						}
			else if (autoSelected == autoTest) {
				switch(autoCounter) {
					case 0: {
						initialAngle = navX->GetAngle();
						autoCounter++;
						leftEncode->Reset();
						break;
					}
					case 1: {
						if (leftEncode->GetDistance() >= 60) {

						}
					}
				}
			}
			else if (autoSelected == autoLeft) {
				switch(autoCounter) {
							case 0: {
								SmartDashboard::PutNumber("Encoder: ", abs(leftEncode->GetDistance()));
								if (abs(leftEncode->GetDistance()) > 118) {
									myRobot->TankDrive(.0,.0);
									autoCounter++;
									leftEncode->Reset();
									leftEncode->SetDistancePerPulse(encodeDistance);
									break;
								} else {
									myRobot->TankDrive(.6,.6);
									break;
								}
							}
							case 1: break;
							/* case 1: {
								SmartDashboard::PutNumber("Angle: ", navX->GetAngle());
								if (abs(navX->GetAngle()) > 42) {
									myRobot->TankDrive(.0,.0);
									autoCounter++;
									break;
								} else {
									myRobot->TankDrive(.4,-.4);
									break;
								}
							}
							case 2: {
								if (abs(leftEncode->GetDistance()) > 15) {
									myRobot->TankDrive(.0,.0);
									autoCounter++;
									break;
								} else {
									myRobot->TankDrive(-.6,-.6);
									break;
								}
							}
							case 3: break; */
						}
			}

	}
	void TeleopInit() {
		directBool = true;
		limitInterlock = true;
		joystick1_1_Inter = true;
		joystick2_1_Inter = true;
		joystick3_1_Inter = true;
		joystick4_1_Inter = true;
		joystick5_1_Inter = true;
		joystick6_1_Inter = true;
		joystick7_1_Inter = true;
		joystick8_1_Inter = true;
		joystick1_2_Inter = true;
		joystick2_2_Inter = true;
		joystick3_2_Inter = true;
		joystick4_2_Inter = true;
		joystick5_2_Inter = true;
		joystick6_2_Inter = true;
		joystick7_2_Inter = true;
		joystick8_2_Inter = true;
		ejectBool = true;
	}

	void TeleopPeriodic() {
		while( IsOperatorControl() && IsEnabled()) {
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
			joystick1_3 = stick3->GetRawButton(1);
			joystickLT_3 = stick3->GetRawAxis(2);
			joystickRT_3 = stick3->GetRawAxis(3);
			joystick6_3 = stick3->GetRawButton(6);
			joystick3_3 = stick3->GetRawButton(3);
			joystick4_3 = stick3->GetRawButton(4);
			joystick2_3 = stick3->GetRawButton(2);
			joystick5_3 = stick3->GetRawButton(5);
			//Need interlock to avoid rapid button switching
			if (joystickLT_3 > 0) {
				agitator->Set(-1);
			}
			else {
				agitator->Set(0);
			}
			if (joystick1_3) {
				shooter->Set(.64);
			}
			else {
				shooter->Set(0);
			}
			if (!joystick1_3) {
				joystick1_3_Inter = true;
			}
			if (joystick3_2 && joystick3_2_Inter) {
				shifting->Set(shifting->kForward);
				joystick3_2_Inter = false;
			}
			if (!joystick3_2) {
				joystick3_2_Inter = true;
			}
			if (joystick4_2 && joystick4_2_Inter) {
				shifting->Set(DoubleSolenoid::kReverse);
				joystick4_2_Inter = false;
			}
			if (!joystick4_2) {
				joystick4_2_Inter = true;
			}
			if (joystick5_3 && gear->Get() == DoubleSolenoid::kForward) {
				gearEject->Set(DoubleSolenoid::kForward);
			}
			else {
				gearEject->Set(DoubleSolenoid::kReverse);
			}
			if (joystick3_3 && joystick3_3_Inter) {
				gear->Set(DoubleSolenoid::kReverse);
				ejectBool = false;
				joystick3_3_Inter = false;
			}
			if (!joystick3_3) {
				joystick3_3_Inter = true;
			}
			if (joystick4_3 && joystick4_3_Inter) {
				gear->Set(DoubleSolenoid::kForward);
				ejectBool = true;
				joystick4_3_Inter = false;
			}
			if (!joystick4_3) {
				joystick4_3_Inter = true;
			}
			if (!joystick5_1) {
				joystick5_1_Inter = true;
			}
			if (joystick2_2 && joystick2_2_Inter) {
				directBool = !directBool;
				joystick2_2_Inter = false;
			}
			if (joystick2_1 && joystick2_2_Inter) {
				joystick2_1_Inter = false;
				leftEncode->Reset();
				while (abs(leftEncode->GetDistance()) < 8) {
					myRobot->TankDrive(-.6,-.6);
				}
			}
			if (!joystick2_1) {
				joystick2_1_Inter = true;
			}
			if (!joystick2_2) {
				joystick2_2_Inter = true;
			}
			if (joystick2_3) {
				climber->Set(.80);
			}
			if (!joystick2_3) {
				climber->Set(0);
			}
			if (!joystick8_2) {
				joystick8_2_Inter = true;
			}
			if (directBool) {
				joyLeftValue = joystickY_1 * -1;
				joyRightValue = joystickY_2 * -1;
			} else {
				joyLeftValue =  joystickY_2;
				joyRightValue = joystickY_1;
			}
			if (joystickRT_3 > 0) {
				intake->Set(-.65);
			}
			else if (joystick6_3) {
				intake->Set(.65);
			}
			else {
				intake->Set(0);
			}

			myRobot->TankDrive(joyLeftValue, joyRightValue, true);
		}
	}

	void TestPeriodic() {
		lw->Run();

	}

private:
	frc::LiveWindow* lw = LiveWindow::GetInstance();
	frc::SendableChooser<std::string> chooser;
	const std::string autoLeft = "Left Gear";
	const std::string autoMiddle = "Middle Gear";
	const std::string autoRight = "Right Gear";
	const std::string shootLeft = "Left Shoot";
	const std::string shootRight = "Right Shoot";
	const std::string driveForward = "Drive Forward";
	const std::string gearPassive = "Passive Gear";
	const std::string autoTest = "Test Auto";
	std::string autoSelected;
};
START_ROBOT_CLASS(Robot)
