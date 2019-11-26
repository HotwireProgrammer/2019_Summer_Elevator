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
import frc.robot.*;

public class LimelightTrack extends AutoStep {

    public Robot robot;
    public Robot.LimelightPlacement placement;
    public int direction;

    public LimelightTrack(DriveTrain driveTrain, Robot robot, Robot.LimelightPlacement placement, int direction) {
        super(driveTrain);
        this.robot = robot;
        this.placement = placement;
        this.direction = direction;
    }

    public void Begin() {
        robot.hitTarget = false;
    }

    public void Update() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
		NetworkTableEntry tx = table.getEntry("tx");
		NetworkTableEntry ty = table.getEntry("ty");
		NetworkTableEntry ta = table.getEntry("ta");
        NetworkTableEntry tv = table.getEntry("tv");
        
		double x = tx.getDouble(0.0);
		double y = ty.getDouble(0.0);
		double area = ta.getDouble(0.0);
        double value = tv.getDouble(0.0);

        if (value == 0){
            driveTrain.SetLeftSpeed(0.35f * direction);
            driveTrain.SetRightSpeed(-0.35f * direction);

        }else{
            if (robot.Limelight(placement)) {
                isDone = true;
            }
        }

       
    }
}