package frc.robot.autostep;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.DriveTrain;

public class Wait extends AutoStep {

    public Timer driveTimer;
    public float length;
    
    public Wait(DriveTrain driveTrain, float length) {
        super(driveTrain);
        this.length = length;
    }

    public void Begin() {
        driveTimer = new Timer();
        driveTimer.reset();
        driveTimer.start();
        driveTrain.SetBothSpeed(0.0f);
    }

    public void Update() {
        if (driveTimer.get() > length) {
            isDone = true;
        }
    }
}