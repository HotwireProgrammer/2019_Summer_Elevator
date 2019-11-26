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
	public Compressor compressor = new Compressor();

	// Drivetrain
	public DriveTrain driveTrain = new DriveTrain(0, 1, 2, 3, null);

	// neumatics

	// Joysticks
	public Joystick driver;
	public Joystick operator;
	public boolean arcadeDrive = false;
	public Joystick flightStickLeft;
	public Joystick flightStickRight;
	public float halfSpeed = 1;

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

	public void robotInit() {
		NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(2);
	}

	public void disabledInit() {
		// Controllers
		driver = new Joystick(0);
		operator = new Joystick(1);
	}

	public void autonomousInit() {
	}

	public void autonomousPeriodic() {
	}

	public void teleopInit() {
		NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(2);

		currentState = RobotState.Teleop;

		// Controllers
		driver = new Joystick(0);
		operator = new Joystick(1);
		flightStickLeft = new Joystick(3);
		flightStickRight = new Joystick(2);
	}

	public void teleopPeriodic() {
		RobotLoop();
	}

	public void testInit() {

		// Controllers
		driver = new Joystick(0);
		operator = new Joystick(1);
		flightStickLeft = new Joystick(3);
		flightStickRight = new Joystick(2);

	}

	public void testPeriodic() {

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

	}

	public void RobotLoop() {
		if (currentState == RobotState.Teleop) {

			// Limelight
			if (operator.getRawButton(5) || operator.getRawButton(6)) {

				if (operator.getRawButtonPressed(5) || operator.getRawButtonPressed(6)) {
					// hitTarget = false;
				}

				if (operator.getRawButton(5)) {
					Limelight(LimelightPlacement.Place);
				} else {
					Limelight(LimelightPlacement.Pickup);
				}

			} else {

				// Turn off limelight
				NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);

			}

		} else if (currentState == RobotState.Autonomous) {
		}
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

			return multiplier * (0.4f * (float) Math.tan(1.8 * (multiplier * ControllerInput) - .9) + 0.5f);

		} else if (selection == DriveScale.cb) {

			return (float) Math.pow(ControllerInput, 3);

		} else if (selection == DriveScale.cbrt) {

			return multiplier * (0.63f * (float) Math.cbrt((multiplier * ControllerInput) - 0.5f) + 0.5f);

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

		Boolean hitTarget = true;
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

					driveTrain.SetLeftSpeed(turnSpeed);
					driveTrain.SetRightSpeed(-turnSpeed);
				} else if (x <= -turnBuffer) {

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

			driveTrain.SetBothSpeed(reverseSpeed);

			return true;
		}

		return false;
	}
}