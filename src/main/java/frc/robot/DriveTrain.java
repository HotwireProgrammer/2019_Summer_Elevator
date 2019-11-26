package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;

public class DriveTrain implements PIDOutput {
	JoshMotorControllor joshmotorcontrollorLeftBottomOne;
	JoshMotorControllor joshmotorcontrollorLeftBottomTwo;
	JoshMotorControllor joshmotorcontrollorRightBottomOne;
	JoshMotorControllor joshmotorcontrollorRightBottomTwo;
	float lerpSpeed = 0.8f;
	public AHRS navx;
	PIDController turnController;

	public DriveTrain(int pwm1, int pwm2, int pwm3, int pwm4, AHRS navx) {
		joshmotorcontrollorLeftBottomOne = new JoshMotorControllor(pwm1, lerpSpeed, false);
		joshmotorcontrollorLeftBottomTwo = new JoshMotorControllor(pwm2, lerpSpeed, false);
		joshmotorcontrollorRightBottomOne = new JoshMotorControllor(pwm3, lerpSpeed, false);
		joshmotorcontrollorRightBottomTwo = new JoshMotorControllor(pwm4, lerpSpeed, true);

		this.navx = navx;
		turnController = new PIDController(5.00, 1.0, 0.00020, 0, this.navx, this);
		turnController.setInputRange(-180.0f, 180.0f);
		turnController.setOutputRange(-1.0f, 1.0f);
		turnController.setAbsoluteTolerance(2.0);
		turnController.setContinuous(true);
		turnController.disable();
	}

	public void Update() {
		joshmotorcontrollorLeftBottomOne.UpdateMotor();
		joshmotorcontrollorLeftBottomTwo.UpdateMotor();
		joshmotorcontrollorRightBottomOne.UpdateMotor();
		joshmotorcontrollorRightBottomTwo.UpdateMotor();
	}

	public void SetLeftSpeed(float Speed) {
		joshmotorcontrollorLeftBottomOne.target = Speed;
		joshmotorcontrollorLeftBottomTwo.target = Speed;
	}

	public void SetRightSpeed(float Speed) {
		joshmotorcontrollorRightBottomOne.target = -Speed;
		joshmotorcontrollorRightBottomTwo.target = -Speed;
	}

	public void SetBothSpeed(float Speed) {
		SetLeftSpeed(Speed);
		SetRightSpeed(Speed);
	}

	public void SetBreak() {
		joshmotorcontrollorRightBottomOne.SetBrake();
		joshmotorcontrollorRightBottomTwo.SetBrake();
		joshmotorcontrollorLeftBottomOne.SetBrake();
		joshmotorcontrollorLeftBottomTwo.SetBrake();
	}

	public void SetCoast() {
		joshmotorcontrollorRightBottomOne.SetCoast();
		joshmotorcontrollorRightBottomTwo.SetCoast();
		joshmotorcontrollorLeftBottomOne.SetCoast();
		joshmotorcontrollorLeftBottomTwo.SetCoast();

	}

	public void DriveStraight(float speed, boolean reverse) {
		float pidError = (float) turnController.get();
		SetLeftSpeed((speed * pidError) + speed); // 0.6972
		SetRightSpeed(((speed) - (speed * pidError)) * -1); // -0.583

		speed = -speed;
		if (reverse) {
			speed = -speed;
		}

		// This stuff here is important!
		System.out.println("STRAIGHT YAW " + navx.getYaw() + ";");
		System.out.println("P: " + turnController.getP() + ";");
		System.out.println("I: " + turnController.getI() + ";");
		System.out.println("D: " + turnController.getD() + ";");
		System.out.println("F: " + turnController.getF() + ";");
	}

	public void ClearRotation() {
		navx.zeroYaw();
		turnController.setSetpoint(0);
	}

	@Override
	public void pidWrite(double output) {
	}

}