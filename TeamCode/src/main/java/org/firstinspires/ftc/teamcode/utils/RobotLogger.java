package org.firstinspires.ftc.teamcode.utils;

import org.firstinspires.ftc.teamcode.Robot;
import org.psilynx.psikit.core.Logger;

public class RobotLogger {
    private final Robot robot;

    private boolean logIntake = false;
    private boolean logShooter = false;
    private boolean logFollower = false;
    private boolean logCurrent = false;

    public RobotLogger(Robot robot) {
        this.robot = robot;

        Logger.recordMetadata("Alliance", robot.alliance.toString());
        Logger.start();
    }

    public void updateIntake() {
        Logger.recordOutput("Intake/State", robot.intake.isActive() ? "Active" : "Inactive");
        Logger.recordOutput("Intake/LeftSensor/Detected", robot.intake.leftDetected);
        Logger.recordOutput("Intake/RightSensor/Detected", robot.intake.rightDetected);
        Logger.recordOutput("Intake/Color/R", robot.intake.Red);
        Logger.recordOutput("Intake/Color/G", robot.intake.Green);
        Logger.recordOutput("Intake/Color/B", robot.intake.Blue);
    }
    public void updateShooter() {
        Logger.recordOutput("Shooter/Active", robot.shooter.isActive());
        Logger.recordOutput("Shooter/TargetRPM", robot.shooter.getTargetRPM());
        Logger.recordOutput("Shooter/CurrentRPM", robot.shooter.getCurrentRPM());
        Logger.recordOutput("Shooter/TargetHoodPosition", robot.shooter.getTargetHoodPosition());
        Logger.recordOutput("Shooter/hoodPosition", robot.shooter.getCurrentHoodPosition());
    }
    public void updateFollower() {
        Logger.recordOutput("Follower/Pose/x", robot.follower.pose().x());
        Logger.recordOutput("Follower/Pose/y", robot.follower.pose().y());
        Logger.recordOutput("Follower/Pose/heading", robot.follower.pose().heading());
    }
    private void updateCurrent() {
        Logger.recordOutput("Current/Voltage", robot.voltage);
        Logger.recordOutput("Current/Flywheel", robot.shooter.getCurrent());
        Logger.recordOutput("Current/Intake", robot.intake.getCurrent());
    }
    public void updateLoopTime() {
        Logger.recordOutput("Frequency (Hz)", robot.getFrequency());
    }

    public void update() {
        if (logIntake) updateIntake();
        if (logShooter) updateShooter();
        if (logFollower) updateFollower();
        if (logCurrent) updateCurrent();
        updateLoopTime();
    }

    public void setIntakeLogging(boolean log) { logIntake = log; }
    public void setShooterLogging(boolean log) { logShooter = log; }
    public void setFollowerLogging(boolean log) { logFollower = log; }
    public void setCurrentLogging(boolean log) { logCurrent = log; }
}
