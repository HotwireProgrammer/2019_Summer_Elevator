package frc.robot.autostep;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.DriveTrain;
import frc.robot.*;

public class TriggerArm extends AutoStep {
    public Robot robot;
    public boolean down;

    public TriggerArm(Robot robot, boolean down) {
        super(null);
        this.robot = robot;
        this.down = down;
    }

    public void Begin() {

    }

    public void Update() {
        robot.DiskBrakeDisable();
        isDone = true;
        robot.armHold = false;
        robot.runArm = true;

        if (down) {
            robot.lowerBuffer = robot.groundTarget;
            robot.upperBuffer = robot.groundTarget - robot.armBuffer;
            robot.potTarget = robot.groundTarget;
        } else {
            robot.lowerBuffer = robot.hatchTarget + robot.armBuffer - 0.01f;
            robot.upperBuffer = robot.hatchTarget - robot.armBuffer - 0.01f;
            robot.potTarget = robot.hatchTarget - 0.01f;
        }
    }
}