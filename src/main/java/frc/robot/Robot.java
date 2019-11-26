package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Talon;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;

import java.nio.Buffer;
import java.rmi.server.Operation;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick.AxisType;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.hal.PDPJNI;
import frc.robot.autostep.*;
import edu.wpi.first.wpilibj.Compressor;

public class Robot extends TimedRobot {

	// Sensors
	public AHRS navx = new AHRS(SPI.Port.kMXP);
	public Ultrasonic ultrasonic = new Ultrasonic(4, 5);
	public HotPot pot = new HotPot(1);
	public DigitalInput intakeLimit = new DigitalInput(9);
	public Compressor compressor = new Compressor();

	// Drivetrain
	public DriveTrain driveTrain = new DriveTrain(0, 1, 2, 3, navx);

	// neumatics
	public DoubleSolenoid hatch = new DoubleSolenoid(5, 4);
	public DoubleSolenoid diskBrake = new DoubleSolenoid(3, 2);
	public boolean buttonReleased;
	public boolean hatchReleased;

	// Joysticks
	public Joystick driver;
	public Joystick operator;
	public Joystick debug;
	public boolean arcadeDrive = false;
	public Joystick flightStickLeft;
	public Joystick flightStickRight;
	public float halfSpeed = 1;

	// Arm
	public TalonSRX armLeft = new TalonSRX(3);
	public TalonSRX armRight = new TalonSRX(4);

	// Arm Variables
	public boolean armHold = false;
	public boolean runArm = true;
	public boolean povReleased = false;
	public double potTarget = 0;
	public double currentBuffer;
	public double armBuffer = 1f;
	public Timer brakeTimer;

	// Arm targets
	public double groundTarget = 113;
	public double shipCargoTarget = 30;
	public double rocketCargoTargetBot = 65;
	public double rocketCargoTargetMid = 60;
	public double hatchTarget = 108;
	public double climbTarget = 0;

	// Climbing Variables
	public float downForce = 0.0f;
	public float ultraheight = 6.5f;
	public double lowerBuffer = 0;
	public double upperBuffer = 0;
	public boolean checkPlatform = false;
	public float holdSpeed = 0.0f;
	public boolean foundPlatform = false;

	// Intake
	public TalonSRX intakeTop = new TalonSRX(1);
	public TalonSRX intakeBottom = new TalonSRX(2);
	public boolean hitTarget = false;

	// Gear rack controllers
	public GearRack gearRackFrontOne = new GearRack("FGR1", 6, -1, 2, 5);
	public GearRack gearRackBackOne = new GearRack("BGR1", 7, -1, 0, -1);
	public GearRack gearRackBackTwo = new GearRack("BGR2", 8, 1, 1, -1);

	enum AutoChoice {
		Left, Right
	}

	enum RobotState {
		Autonomous, Teleop;
	}

	public enum DriveScale {
		linear, parabala, tangent, cb, cbrt, 
	}

	public RobotState currentState;

	public enum LimelightPlacement {
		Place, Pickup
	};

	// Auto
	public AutoStep[] autonomous;
	public int currentAutoStep;

	public void robotInit() {
		ultrasonic.setAutomaticMode(true);
		NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(2);
	}

	public void disabledInit() {
		// Controllers
		driver = new Joystick(0);
		operator = new Joystick(1);
		DiskBrakeEnable();
	}

	public void autonomousInit() {
		NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(1);

		driveTrain.SetBreak();
		ArmMove(0);
		brakeTimer = new Timer();
		brakeTimer.start();

		currentState = RobotState.Autonomous;

		currentAutoStep = 0;
		autonomous = new AutoStep[21];

		/*
		 * // Auto off level 2
		 * 
		 * autonomous[1] = new TimedForward(driveTrain, 0.8f, 0.9f); autonomous[2] = new
		 * TriggerArm(this, false); autonomous[3] = new TimedTurn(driveTrain, 0.1f,
		 * 0.5f); autonomous[4] = new NavxTurn(driveTrain, navx, 15f, 0.5f);
		 * autonomous[5] = new TimedForward(driveTrain, 0.65f, 0.5f); autonomous[6] =
		 * new NavxTurn(driveTrain, navx, 5f, 0.5f);
		 */

		AutoChoice autoChoice = AutoChoice.Left;
		String auto = SmartDashboard.getString("autoSelect", "left");
		if (auto.equals("left")) {
			autoChoice = AutoChoice.Left;
		} else if (auto.equals("right")) {
			autoChoice = AutoChoice.Right;
		}
		System.out.println("AUTO CHOICE " + autoChoice + " networkTables " + auto);

		// Auto off level 1 left
		autonomous[0] = new TriggerArm(this, false);
		autonomous[1] = new NavxReset(driveTrain, navx);
		autonomous[2] = new TimedForward(driveTrain, 1.2f, 0.6f);
		autonomous[3] = new NavxTurn(driveTrain, navx, 0.0f, 0.6f);
		autonomous[4] = new Wait(driveTrain, 1.0f);
		autonomous[5] = new LimelightTrack(driveTrain, this, LimelightPlacement.Place, 0);
		autonomous[6] = new Wait(driveTrain, 0.3f);
		autonomous[7] = new TimedForward(driveTrain, 0.22f, -0.8f);
		autonomous[8] = new TriggerArm(this, false);

		if (autoChoice == AutoChoice.Left) {
			autonomous[9] = new NavxTurn(driveTrain, navx, -100.0f, 0.5f);
			autonomous[10] = new TimedForward(driveTrain, 0.98f, 1.0f);
		} else {
			autonomous[9] = new NavxTurn(driveTrain, navx, 100.0f, 0.5f);
			autonomous[10] = new TimedForward(driveTrain, 0.98f, 1.0f);
		}
		autonomous[11] = new Wait(driveTrain, 0.2f);

		if (autoChoice == AutoChoice.Left) {
			autonomous[12] = new LimelightTrack(driveTrain, this, LimelightPlacement.Pickup, -1);
		} else {
			autonomous[12] = new LimelightTrack(driveTrain, this, LimelightPlacement.Pickup, 1);
		}

		autonomous[13] = new Wait(driveTrain, 0.5f);
		autonomous[14] = new TimedForward(driveTrain, 0.5f, -0.8f);
		autonomous[15] = new TriggerArm(this, false);
		autonomous[16] = new TimedForward(driveTrain, 0.4f, -0.8f);

		if (autoChoice == AutoChoice.Left) {
			autonomous[17] = new NavxTurn(driveTrain, navx, -60.0f, 0.6f);
		} else {
			autonomous[17] = new NavxTurn(driveTrain, navx, 60.0f, 0.6f);
		}

		autonomous[18] = new TimedForward(driveTrain, 0.60f, 0.75f);

		if (autoChoice == AutoChoice.Left) {
			autonomous[19] = new LimelightTrack(driveTrain, this, LimelightPlacement.Place, -1);
		} else {
			autonomous[19] = new LimelightTrack(driveTrain, this, LimelightPlacement.Place, 1);
		}
		autonomous[20] = new TimedForward(driveTrain, 0.5f, -0.4f);

		// auto right

		autonomous[0].Begin();
	}

	public void autonomousPeriodic() {
		RobotLoop();
	}

	public void teleopInit() {
		NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(2);

		brakeTimer = new Timer();
		brakeTimer.start();

		currentState = RobotState.Teleop;

		checkPlatform = false;
		foundPlatform = false;
		ultrasonic.setAutomaticMode(true);

		armHold = true;

		// Controllers
		debug = new Joystick(3);
		driver = new Joystick(0);
		operator = new Joystick(1);
		flightStickLeft = new Joystick(3);
		flightStickRight = new Joystick(2);
	}

	public void teleopPeriodic() {
		RobotLoop();
	}

	public void testInit() {

		navx.reset();

		ultrasonic.setAutomaticMode(true);

		// Controllers
		debug = new Joystick(3);
		driver = new Joystick(0);
		operator = new Joystick(1);
		flightStickLeft = new Joystick(3);
		flightStickRight = new Joystick(2);

		DiskBrakeDisable();
		HatchHold();
	}

	public void testPeriodic() {

		// System.out.println("Encoder Back Two : " +
		// gearRackBackTwo.GetEncoderPosition());
		// System.out.println("Encoder Back One: " +
		// gearRackBackOne.GetEncoderPosition());
		// System.out.println("Encoder Front One: " +
		// gearRackFrontOne.GetEncoderPosition());
		// System.out.println("Encoder Front Two: " +
		// gearRackFrontTwo.GetEncoderPosition());
		// System.out.println("Ultrasonic: " + ultrasonic.getRangeInches());

		// gearRackBackOne.Write();
		// gearRackFrontOne.Write();
		// gearRackBackTwo.Write();
		// gearRackFrontTwo.Write();

		// System.out.println("Limit: " + intakeLimit.get());
		// System.out.println("Navx Yaw:" + navx.getYaw());
		// System.out.println("Pot: " + pot.get());
		NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
		NetworkTableEntry tx = table.getEntry("tx");
		NetworkTableEntry ty = table.getEntry("ty");
		NetworkTableEntry ta = table.getEntry("ta");

		double x = tx.getDouble(0.0);
		double y = ty.getDouble(0.0);
		double area = ta.getDouble(0.0);

		NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);

		ControllerDrive();
		UpdateMotors();

		if (driver.getRawButton(2)) {
			ArmMove(0.5f);
		} else if (driver.getRawButton(3)) {
			ArmMove(-0.5f);
		} else {
			ArmMove(0.0f);
		}
	}

	public void RobotLoop() {
		// System.out.println("Pot: " + pot.get());
		gearRackBackOne.Write();
		gearRackBackTwo.Write();
		gearRackFrontOne.Write();
		SmartDashboard.putNumber("UltrasonicDown", ultrasonic.getRangeInches());
		SmartDashboard.putNumber("Pot Value", pot.get());
		SmartDashboard.putNumber("Navx Value", navx.getYaw());

		if (currentState == RobotState.Teleop) {

			// System.out.println("WORKING");

			// Climbing

			driver.setRumble(RumbleType.kLeftRumble, 0);

			if (operator.getRawButtonPressed(7)) {

				gearRackBackOne.ResetEncoder();
				gearRackBackTwo.ResetEncoder();
				gearRackFrontOne.ResetEncoder();

				foundPlatform = false;
			}

			if (operator.getRawButtonPressed(8)) {
				ArmMove(0.0f);
				runArm = false;
				DiskBrakeDisable();

			}
			if (operator.getRawButton(10)) {
				ArmMove(0.0f);
				float backSpeed = 0.4f;
				gearRackBackOne.SetMotorSpeed(backSpeed);
				gearRackBackTwo.SetMotorSpeed(backSpeed);
			}

			if (operator.getRawButton(7) && operator.getRawButton(8)) {
				compressor.stop();
				armHold = false;
				runArm = false;
				DiskBrakeDisable();

				if (!foundPlatform) {
					if (ultrasonic.getRangeInches() > 15) {
						checkPlatform = true;
						ArmMove(0.0f);
						Outtake(0.8f);
						driveTrain.SetBothSpeed(0.7f);
					} else {
						Intake(0.0f);
						driveTrain.SetLeftSpeed(0.0f);
						driveTrain.SetRightSpeed(0.0f);
					}

					float backSpeed = 0.325f;
					float frontSpeed = 0.6f;
					float rollTarget = -5.5f;
					float rollAdjustment = 0.25f;

					if (navx.getRoll() < rollTarget) {
						backSpeed = backSpeed + rollAdjustment;
						System.out.println("Adjusting Roll up");
					} else if (navx.getRoll() > -rollTarget) {
						backSpeed = backSpeed - rollAdjustment;
						System.out.println("Adjusting Roll down");
					}

					gearRackFrontOne.SetMotorSpeed(frontSpeed);
					gearRackBackOne.SetMotorSpeed(backSpeed);
					gearRackBackTwo.SetMotorSpeed(backSpeed);

					System.out.println("Going up");

					if (checkPlatform) {
						System.out.println("Checking for platform");
						float platformUltraHeight = 5;
						if (ultrasonic.getRangeInches() < platformUltraHeight) {
							foundPlatform = true;
						}
					}
				}

				if (foundPlatform) {

					System.out.println("Found Platform");

					if (operator.getRawAxis(3) > 0.1) {
						driveTrain.SetBothSpeed(0.0f);

						gearRackBackOne.SetMotorSpeed(-0.5);
						gearRackBackTwo.SetMotorSpeed(-0.5);
					} else {

						int runningUpStopPosition = 100;
						if (gearRackFrontOne.GetEncoderPosition() > runningUpStopPosition) {
							gearRackFrontOne.SetMotorSpeed(1.0f);
						} else {
							gearRackFrontOne.SetMotorSpeed(0.0f);
						}

						if (gearRackFrontOne.GetEncoderPosition() <= runningUpStopPosition) {
							driveTrain.SetBothSpeed(0.3f);
						} else {
							driveTrain.SetBothSpeed(0.0f);
						}

						ArmMove(0.0f);
					}
				}

			} else {

				if (flightStickLeft.getRawButton(3) || flightStickRight.getRawButton(3)) {
					if (ultrasonic.getRangeInches() < 5) {
						driveTrain.SetBothSpeed(0.08f);
					} else {
						driveTrain.SetBothSpeed(0.0f);
					}
				} else {
					ControllerDrive();
				}

				// Arm Targets
				if (povReleased == true && (operator.getPOV() != -1 || operator.getRawAxis(2) != 0)
						|| operator.getRawAxis(3) != 0) {
					povReleased = false;
					armHold = false;
					runArm = true;
					DiskBrakeDisable();

					if (operator.getPOV() == 0) {
						lowerBuffer = rocketCargoTargetBot + armBuffer;
						upperBuffer = rocketCargoTargetBot - armBuffer;
						potTarget = rocketCargoTargetBot;

					} else if (operator.getPOV() == 180) {
						lowerBuffer = groundTarget;
						upperBuffer = groundTarget - armBuffer;
						potTarget = groundTarget;

					} else if (operator.getPOV() == 90) {
						lowerBuffer = shipCargoTarget + armBuffer;
						upperBuffer = shipCargoTarget - armBuffer;
						potTarget = shipCargoTarget;

					} else if (operator.getPOV() == 270) {
						lowerBuffer = hatchTarget + armBuffer;
						upperBuffer = hatchTarget - armBuffer;
						potTarget = hatchTarget;

					} else if (operator.getRawAxis(2) > 0.0f) {
						lowerBuffer = rocketCargoTargetMid + armBuffer;
						upperBuffer = rocketCargoTargetMid - armBuffer;
						potTarget = rocketCargoTargetMid;

					} else if (operator.getRawAxis(3) > 0.0f) {
						lowerBuffer = climbTarget + armBuffer;
						upperBuffer = climbTarget - armBuffer;
						potTarget = climbTarget;

					}
				}
				if (operator.getPOV() == -1) {
					povReleased = true;
				}

				RunArmControls();

				// Hatch
				if (operator.getRawButtonPressed(3)) {
					if (hatchReleased) {
						HatchHold();
					} else {
						HatchRelease();
					}
				}

				// Limelight
				if (operator.getRawButton(5) || operator.getRawButton(6)) {

					if (operator.getRawButtonPressed(5) || operator.getRawButtonPressed(6)) {
						hitTarget = false;
					}

					if (operator.getRawButton(5)) {
						Limelight(LimelightPlacement.Place);
					} else {
						Limelight(LimelightPlacement.Pickup);
					}

				} else {

					// Turn off limelight
					NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);

					// disbrake manual
					// TODO delete me

					if (operator.getRawButtonPressed(2)) {
						DiskBrakeEnable();
					}

					// Intake
					if (operator.getRawButton(1)) {
						if (intakeLimit.get() == false) {
							Intake(0.9f);
						} else {
							Intake(0.0f);
						}
					} else if (operator.getRawButton(4)) {
						if (potTarget == rocketCargoTargetBot) {
							Outtake(0.6f);
						} else if (potTarget == rocketCargoTargetMid) {
							intakeTop.set(ControlMode.PercentOutput, -0.85f);
							intakeBottom.set(ControlMode.PercentOutput, -0.2f);
						} else {
							intakeTop.set(ControlMode.PercentOutput, -0.5f);
							intakeBottom.set(ControlMode.PercentOutput, 0.0f);
						}
					} else {
						Outtake(0.0f);
					}
				}

				float backSpeed = -0.7f; // 0.4
				if (operator.getRawAxis(1) < -0.8f) {
					gearRackBackOne.SetMotorSpeed(-backSpeed);
					gearRackBackTwo.SetMotorSpeed(-backSpeed);
				} else if (operator.getRawAxis(1) > 0.8f) {
					gearRackBackOne.SetMotorSpeed(backSpeed);
					gearRackBackTwo.SetMotorSpeed(backSpeed);
				} else {
					gearRackBackOne.SetMotorSpeed(0);
					gearRackBackTwo.SetMotorSpeed(0);
					gearRackFrontOne.SetMotorSpeed(0);
				}

				if (operator.getRawAxis(5) < -0.8f) {
					runArm = false;
					ArmMove(0.65f);
					DiskBrakeDisable();
				} else if (operator.getRawAxis(5) > 0.8f) {
					runArm = false;
					ArmMove(-0.65f);
					DiskBrakeDisable();
				} else if (!runArm) {
					ArmMove(0f);
				}

				if (operator.getRawAxis(1) > 0.8 || operator.getRawAxis(1) < -0.8 || operator.getRawAxis(5) > 0.8
						|| operator.getRawAxis(5) < -0.8) {
					compressor.stop();
				} else {
					compressor.start();
				}
				if (flightStickLeft.getRawButton(2) || flightStickRight.getRawButton(2)) {
					Intake(-0.5f);
				}
			}

		} else if (currentState == RobotState.Autonomous) {

			if (operator.getRawButton(9) && operator.getRawButton(10)) {
				currentState = RobotState.Teleop;
			}

			System.out.println("Current auto step " + currentAutoStep);
			if (currentAutoStep < autonomous.length) {

				autonomous[currentAutoStep].Update();

				if (autonomous[currentAutoStep].isDone) {
					currentAutoStep = currentAutoStep + 1;
					if (currentAutoStep < autonomous.length) {
						autonomous[currentAutoStep].Begin();
					}
				}
			} else {
				System.out.println("Autonomous Done");
				driveTrain.SetBothSpeed(0.0f);
				// currentState = RobotState.Teleop;
			}

			RunArmControls();
		}

		UpdateMotors();
	}

	public void UpdateMotors() {
		driveTrain.Update();
	}

	public float TranslateController(float input) {
		float deadzone = 0.15f;
		if (input > -deadzone && input < deadzone) {
			input = 0.0f;
		}
		float a = 0.7f;
		float output = (a * input * input * input) + (1 - a) * input;
		return output;
	}

	public float DriveScaleSelector(float ControllerInput, DriveScale selection) {
		float multiplier = (ControllerInput / (float) Math.abs(ControllerInput));
		
		if (selection == DriveScale.parabala) {
			return multiplier * (float) Math.pow(ControllerInput, 2);

		} else if (selection == DriveScale.tangent) {

			return multiplier * (0.4f * (float) Math.tan(1.8 * (multiplier*ControllerInput) - .9) + 0.5f);
	
		} else if (selection == DriveScale.cb) {

			return (float) Math.pow(ControllerInput, 3);

		}  else if (selection == DriveScale.cbrt) {

			return multiplier*(0.63f * (float) Math.cbrt((multiplier*ControllerInput) - 0.5f) + 0.5f);

		} else {

			return ControllerInput;

		}
	}

	public void ControllerDrive() {
		
		if (flightStickLeft.getRawButtonPressed(1) || flightStickRight.getRawButtonPressed(1)) {

			if (halfSpeed == 1) {

				halfSpeed = 0.5f;

			} else {

				halfSpeed = 1;

			}

		}
		
		if (arcadeDrive) {
			float rawH = DriveScaleSelector(TranslateController((float) driver.getRawAxis(4)), DriveScale.linear);
			float rawV = DriveScaleSelector(TranslateController((float) driver.getRawAxis(1)), DriveScale.linear);
			// Arcade
			float horJoystick = (rawH / Math.abs(rawH)) * (float) Math.pow(rawH, 2); // 0 4
			float verJoystick = (rawV / Math.abs(rawV)) * (rawV * rawV); // 5 1

			if (Float.isNaN(horJoystick))
				horJoystick = 0;
			if (Float.isNaN(verJoystick))
				verJoystick = 0;

			System.out.println(horJoystick + " - " + verJoystick);

			driveTrain.SetRightSpeed((-verJoystick + -horJoystick) * halfSpeed);
			driveTrain.SetLeftSpeed((-verJoystick + horJoystick) * halfSpeed);
			driveTrain.SetCoast();
		} else {
			// tank
			// float leftJoystick = ((float) flightStickLeft.getRawAxis(1))*((float)
			// flightStickLeft.getRawAxis(1));
			// float rightJoystick = ((float) flightStickRight.getRawAxis(1))*((float)
			// flightStickRight.getRawAxis(1));
			// float leftJoystick = TranslateController((float) driver.getRawAxis(1)); // 0
			// 4
			// float rightJoystick = TranslateController((float) driver.getRawAxis(5)); // 5
			// 1

			float leftJoystick = DriveScaleSelector((float) flightStickLeft.getRawAxis(1), DriveScale.linear);
			float rightJoystick = DriveScaleSelector((float) flightStickRight.getRawAxis(1), DriveScale.linear);

			driveTrain.SetRightSpeed(-rightJoystick * halfSpeed);
			driveTrain.SetLeftSpeed(-leftJoystick * halfSpeed);
			driveTrain.SetCoast();
		}
	}

	public void Intake(float speeed) {
		intakeTop.set(ControlMode.PercentOutput, speeed);
		intakeBottom.set(ControlMode.PercentOutput, speeed);
	}

	public void Outtake(float speeed) {
		intakeTop.set(ControlMode.PercentOutput, -speeed);
		intakeBottom.set(ControlMode.PercentOutput, -speeed);
	}

	public void HatchRelease() {
		hatch.set(DoubleSolenoid.Value.kReverse);
		hatchReleased = true;
	}

	public void HatchHold() {
		hatch.set(DoubleSolenoid.Value.kForward);
		hatchReleased = false;
	}

	public void ArmMove(float speeed) {
		armLeft.set(ControlMode.PercentOutput, -speeed);
		armRight.set(ControlMode.PercentOutput, speeed);
	}

	public void DrivetrainBrakes(boolean brakes) {
		if (brakes = true) {
			driveTrain.SetBreak();
		} else {
			driveTrain.SetCoast();
		}
	}

	public void DiskBrakeEnable() {
		diskBrake.set(DoubleSolenoid.Value.kReverse);
	}

	public void DiskBrakeDisable() {
		diskBrake.set(DoubleSolenoid.Value.kForward);
	}

	public void RunArmControls() {
		// System.out.println("Lower " + lowerBuffer);
		// System.out.println("Upper " + upperBuffer);

		if (runArm) {
			if (!armHold) {

				// NOTE these downForces go from ground up.

				if (pot.get() > lowerBuffer) {
					// moving up

					DiskBrakeDisable();
					// ArmMove(0.25f);

					float upForce = 0.0f;
					if (pot.get() < 15) {
						upForce = 0.2f;
					} else if (pot.get() < 20) {
						upForce = 0.4f;
					} else {
						upForce = 0.35f;
					}

					if (potTarget == shipCargoTarget) {
						upForce += 0.2f;
					}
					if (potTarget == climbTarget) {
						upForce += 0.1f;
					}
					ArmMove(upForce);

				} else if (pot.get() < upperBuffer) {
					// moving down

					DiskBrakeDisable();

					float downForce = 0.0f;
					if (pot.get() < 14) {
						downForce = -0.2f;
					} else if (pot.get() < 20) {
						downForce = -0.15f;
					} else {
						downForce = -0.1f;
					}

					ArmMove(downForce);

				} else {
					armHold = true;
					brakeTimer = new Timer();
					brakeTimer.start();
				}
			}

			if (armHold) {

				DiskBrakeEnable();
				if (brakeTimer.get() > 0.1) {
					ArmMove(0.1f);
				} else {
					ArmMove(0.1f);
				}
			}
		}
	}

	public boolean Limelight(LimelightPlacement placement) {

		NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
		NetworkTableEntry tx = table.getEntry("tx");
		NetworkTableEntry ty = table.getEntry("ty");
		NetworkTableEntry ta = table.getEntry("ta");
		NetworkTableEntry tv = table.getEntry("tv");

		double x = tx.getDouble(0.0);
		double y = ty.getDouble(0.0);
		double area = ta.getDouble(0.0);
		double value = tv.getDouble(0.0);

		// driver.setRumble(RumbleType.kLeftRumble, 1);
		boolean isPlacing = (placement == LimelightPlacement.Place);

		driveTrain.SetBreak();

		// turning
		float turnBufferPlace = 1.2f;
		float turnBufferPickup = 1.9f;
		double turningFarDist = 25;
		double turningSpeedMinimum = 0.35f;
		double maxTurnSpeed = 0.35f;

		// approach
		float approachTargetPlace = 4.7f;// 4.7f
		float approachTargetPickup = 3.8f;
		float approachCloseTA = 0.9f;
		float approachFarTA = 0.162f;
		float approachSpeedClose = 0.3f;
		float approachSpeedFar = 0.6f;

		// reverse
		float reverseSpeed = -0.5f;
		float stopArea = 1.7f;

		potTarget = hatchTarget;

		if (!hitTarget) {

			NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);

			if (value == 0) {
				// System.out.println("no target");
				driveTrain.SetBothSpeed(0.0f);

			} else {
				double normalized = Math.abs(maxTurnSpeed * (x / turningFarDist));
				float turnSpeed = (float) Math.max(normalized, turningSpeedMinimum);

				float turnBuffer = 0.0f;
				if (isPlacing) {
					turnBuffer = turnBufferPlace;
				} else {
					turnBuffer = turnBufferPickup;
				}

				if (x >= turnBuffer) {

					if (isPlacing) {
						HatchHold();
					} else {
						HatchRelease();
					}

					System.out.println("Limelight Turning Right");

					driveTrain.SetLeftSpeed(turnSpeed);
					driveTrain.SetRightSpeed(-turnSpeed);
				} else if (x <= -turnBuffer) {

					if (isPlacing) {
						HatchHold();
					} else {
						HatchRelease();
					}
					System.out.println("Limelight Turning Left");

					driveTrain.SetLeftSpeed(-turnSpeed);
					driveTrain.SetRightSpeed(turnSpeed);
				} else {

					// On target so drive forward
					float approachTarget = 0.0f;
					if (isPlacing) {
						approachTarget = approachTargetPlace;
					} else {
						approachTarget = approachTargetPickup;
					}
					if (area < approachTarget) {

						if (isPlacing) {
							HatchHold();
						} else {
							HatchRelease();
						}

						// number between 0 and 1. 0 is lowest speed, 1 is quickest
						double normalizedApproachDist = (area - approachFarTA) / (approachCloseTA - approachFarTA);
						normalizedApproachDist = Math.max(0.0, normalizedApproachDist);
						normalizedApproachDist = Math.min(1.0, normalizedApproachDist);
						normalizedApproachDist = 1.0 - normalizedApproachDist;

						double approachSpeed = approachSpeedClose
								+ ((approachSpeedFar - approachSpeedClose) * normalizedApproachDist);

						driveTrain.SetBothSpeed((float) approachSpeed);
					} else {
						hitTarget = true;
					}
				}
			}
		} else {
			if (isPlacing) {
				HatchRelease();
			} else {
				HatchHold();
			}

			driveTrain.SetBothSpeed(reverseSpeed);

			return true;
		}

		return false;
	}
}