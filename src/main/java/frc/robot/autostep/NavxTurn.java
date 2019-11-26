package frc.robot.autostep;

import com.kauailabs.navx.frc.AHRS;
import frc.robot.DriveTrain;

public class NavxTurn extends AutoStep {

    public AHRS navx;
    public float turnDegree;
    public float speed;

    public NavxTurn(DriveTrain driveTrain, AHRS navx, float turnDegree, float speed) {
        super(driveTrain);
        this.navx = navx;
        this.speed = speed;
        this.turnDegree = turnDegree;
    }

    public void Begin() {
        float diff = (turnDegree - navx.getYaw());
        int direction = (int) ((Math.abs(diff)) / diff);

        driveTrain.SetLeftSpeed(speed * direction);
        driveTrain.SetRightSpeed(-speed * direction);
    }

    public void Update() {

        float degreeDifference = Math.abs(navx.getYaw() - turnDegree);
        float goodEnoughDeg = 5.0f;
        if (degreeDifference < goodEnoughDeg) {
            isDone = true;
            driveTrain.SetLeftSpeed(0);
            driveTrain.SetRightSpeed(0);
        }
    }
}