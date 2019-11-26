package frc.robot.autostep;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.DriveTrain;

public abstract class AutoStep {

    public boolean isDone;
    public DriveTrain driveTrain;

    public AutoStep(DriveTrain driveTrain)
    {
        isDone = false;

        this.driveTrain = driveTrain;
    }

    public abstract void Begin();
    public abstract void Update();
}