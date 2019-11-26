package frc.robot.autostep;

import edu.wpi.first.wpilibj.Timer;
import com.kauailabs.navx.frc.AHRS;

import frc.robot.DriveTrain;

public class NavxReset extends AutoStep {

    public AHRS navx;
    public float resetTimeLengthSeconds;
    public Timer navxTime;

    public NavxReset(DriveTrain driveTrain, AHRS navx) {
        super(driveTrain);
        this.navx = navx;
    }

    public void Begin() {
        navxTime = new Timer();
        navxTime.reset();
        navxTime.start();
        navx.reset();
    }

    public void Update() {
        if (navxTime.get() > 0.2) {
            isDone = true;
        }
    }
}