package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.hal.PDPJNI;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PIDController;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class GearRack {

    public TalonSRX motor;
    public TalonSRX motorTwo;

    public long encoderZero;

    public String name;
    public int direction;
    public int limitPort;
    public DigitalInput limit;

    public GearRack(String name, int motorId, int direction, int limitPort, int motorTwoId) {

        limit = new DigitalInput(limitPort);
        this.name = name;
        this.direction = direction;

        motor = new TalonSRX(motorId);
        if (motorTwoId > 0) {
            motorTwo = new TalonSRX(motorTwoId);
        }
    }

    public void Write() {

        // double current = PDPJNI.getPDPChannelCurrent((byte) 5, pdpHandle);
        // SmartDashboard.putNumber(name + "Current", current);

        // SmartDashboard.putNumber(name, GetEncoderPosition());
        SmartDashboard.putNumber(name + " Output", (float) motor.getMotorOutputPercent());
        // SmartDashboard.putNumber(name + " TEST", 0.5);
        SmartDashboard.putNumber(name + " raw encoder ", GetEncoderPosition());
        // SmartDashboard.putNumber(name + " Speed ",
        // motor.getSelectedSensorVelocity());
        SmartDashboard.putBoolean(name + " Limit", limit.get());

    }

    public void ResetEncoder() {
        encoderZero = motor.getSelectedSensorPosition();
    }

    public long GetEncoderPosition() {
        return (encoderZero - motor.getSelectedSensorPosition()) * direction;
    }

    public void SetMotorSpeed(double Speed) {

        boolean movingUp = false;
        if (direction > 0) {
            movingUp = Speed * direction <= 0;
        } else {
            movingUp = Speed * direction >= 0;
        }

        // if (limit == null || !limit.get() || movingUp) {
            motor.set(ControlMode.PercentOutput, Speed * direction);
            if (motorTwo != null) {
                motorTwo.set(ControlMode.PercentOutput, Speed * direction * -1);
            }
        // } else {
        //     System.out.println(name + " Stopped");

        //     motor.set(ControlMode.PercentOutput, 0);
        //     if (motorTwo != null) {
        //         motorTwo.set(ControlMode.PercentOutput, 0);
        //     }
        // }
    }
}
