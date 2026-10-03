package org.firstinspires.ftc.teamcode.utils;

import org.firstinspires.ftc.teamcode.Robot;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.ftc.PedroFollowerOdometryLogger;

public class RobotLogger {
    private final Robot robot;

    private boolean logIntake = false;
    private boolean logShooter = false;
    private boolean logTurret = false;
    private boolean logScorer = false;
    private boolean logFollower = false;
    private boolean logCurrent = false;

    public RobotLogger(Robot robot) {
        this.robot = robot;
    }

    private void updateIntake() {
        Logger.recordOutput("Intake/State", robot.intake.isActive() ? "Active" : "Inactive");
        Logger.recordOutput("Intake/LeftSensor/Detected", robot.intake.leftDetected);
        Logger.recordOutput("Intake/RightSensor/Detected", robot.intake.rightDetected);
        Logger.recordOutput("Intake/Color/R", robot.intake.red);
        Logger.recordOutput("Intake/Color/G", robot.intake.green);
        Logger.recordOutput("Intake/Color/B", robot.intake.blue);
    }
    private void updateShooter() {
        Logger.recordOutput("Shooter/Active", robot.shooter.isActive());
        Logger.recordOutput("Shooter/TargetRPM", robot.shooter.getTargetRPM());
        Logger.recordOutput("Shooter/CurrentRPM", robot.shooter.getCurrentRPM());
        Logger.recordOutput("Shooter/TargetHoodPosition", robot.shooter.getTargetHoodPosition());
        Logger.recordOutput("Shooter/hoodPosition", robot.shooter.getCurrentHoodPosition());
    }
    private void updateTurret() {
        Logger.recordOutput("Turret/TargetAngle", robot.turret.getTargetAngleDegrees());
        Logger.recordOutput("Turret/CurrentAngle", robot.turret.getCurrentAngleDegrees());
        Logger.recordOutput("Turret/EstimatedAngle", robot.turret.getEstimatedAngle());
        Logger.recordOutput("Turret/Ready", robot.turret.isReady());
    }
    private void updateScorer() {
        Logger.recordOutput("Scorer/DistanceToHive", robot.scorer.getDistanceCm());
        Logger.recordOutput("Scorer/TurretAngle", robot.scorer.getTurretAngleDeg());
        Logger.recordOutput("Scorer/VirtualRobotPosition/x", robot.scorer.getVirtualPosition().elements[0]);
        Logger.recordOutput("Scorer/VirtualRobotPosition/y", robot.scorer.getVirtualPosition().elements[1]);
        Logger.recordOutput("Scorer/HivePosition/x", robot.scorer.getHivePosition().elements[0]);
        Logger.recordOutput("Scorer/HivePosition/y", robot.scorer.getHivePosition().elements[1]);
    }
    private void updateFollower() {
        Logger.recordOutput("Follower/Pose/x", robot.follower.pose().x());
        Logger.recordOutput("Follower/Pose/y", robot.follower.pose().y());
        Logger.recordOutput("Follower/Pose/heading", robot.follower.pose().heading());
        PedroFollowerOdometryLogger.log("follower", robot.follower.pose().x(), robot.follower.pose().y(), robot.follower.pose().heading());
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
        Logger.recordOutput("Alliance", robot.alliance.toString());

        if (logIntake) updateIntake();
        if (logShooter) updateShooter();
        if (logTurret) updateTurret();
        if (logScorer) updateScorer();
        if (logFollower) updateFollower();
        if (logCurrent) updateCurrent();
        updateLoopTime();
    }

    public void setIntakeLogging(boolean log) { logIntake = log; }
    public void setShooterLogging(boolean log) { logShooter = log; }
    public void setTurretLogging(boolean log) { logTurret = log; }
    public void setScorerLogging(boolean log) { logScorer = log; }
    public void setFollowerLogging(boolean log) { logFollower = log; }
    public void setCurrentLogging(boolean log) { logCurrent = log; }

}
