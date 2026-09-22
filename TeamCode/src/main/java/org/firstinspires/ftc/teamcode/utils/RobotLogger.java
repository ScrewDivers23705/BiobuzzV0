package org.firstinspires.ftc.teamcode.utils;

import org.firstinspires.ftc.teamcode.Robot;
import org.psilynx.psikit.core.Logger;

public class RobotLogger {
    private final Robot robot;

    private boolean logIntake = false;
    private boolean logShooter = false;
    private boolean logFollower = true;

    public RobotLogger(Robot robot) {
        this.robot = robot;

        Logger.recordMetadata("Alliance", robot.alliance.toString());
        Logger.start();
    }

    public void updateIntake() {
        //Logger.recordOutput("Intake/", robot.intake...);
    }
    public void updateShooter() {
        //Logger.recordOutput("Shooter/", robot.shooter...);
    }
    public void updateFollower() {
        Logger.recordOutput("Follower/Pose/x", robot.follower.pose().x());
        Logger.recordOutput("Follower/Pose/y", robot.follower.pose().y());
        Logger.recordOutput("Follower/Pose/heading", robot.follower.pose().heading());
    }
    public void updateLoopTime() {
        Logger.recordOutput("Frequency (Hz)", robot.getFrequency());
    }

    public void update() {
        if (logIntake) updateIntake();
        if (logShooter) updateShooter();
        if (logFollower) updateFollower();
        updateLoopTime();
    }

    public void setIntakeLogging(boolean log) { logIntake = log; }
    public void setShooterLogging(boolean log) { logShooter = log; }
    public void setFollowerLogging(boolean log) { logFollower = log; }

}
