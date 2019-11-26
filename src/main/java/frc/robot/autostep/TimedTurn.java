package frc.robot.autostep;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.DriveTrain;

public class TimedTurn extends AutoStep {

    public Timer turnTime;
    public float time;
    public float speed;

    public TimedTurn(DriveTrain driveTrain, float time, float speed) {
        super(driveTrain);
        this.time = time;
        this.speed = speed;
        turnTime = new Timer();
    }

    public void Begin() {
        turnTime.reset();
        turnTime.start();
        driveTrain.SetLeftSpeed(speed);
        driveTrain.SetRightSpeed(-speed);
    }

    public void Update() {
        if (turnTime.get() > time) {
            isDone = true;
            driveTrain.SetLeftSpeed(0);
            driveTrain.SetRightSpeed(0);
        }
    }
}