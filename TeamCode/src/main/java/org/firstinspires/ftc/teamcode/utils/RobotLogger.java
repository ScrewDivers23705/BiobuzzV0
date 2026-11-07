package org.firstinspires.ftc.teamcode.utils;

import org.firstinspires.ftc.teamcode.Robot;
import gay.zharel.fateweaver.telemetry.Fate;

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
        Fate.log("Intake/State", robot.intake.isActive() ? "Active" : "Inactive");
        Fate.log("Intake/LeftSensor/Detected", robot.intake.leftDetected);
        Fate.log("Intake/RightSensor/Detected", robot.intake.rightDetected);
        Fate.log("Intake/Color/R", robot.intake.red);
        Fate.log("Intake/Color/G", robot.intake.green);
        Fate.log("Intake/Color/B", robot.intake.blue);
    }
    private void updateShooter() {
        Fate.log("Shooter/Active", robot.shooter.isActive());
        Fate.log("Shooter/TargetRPM", robot.shooter.getTargetRPM());
        Fate.log("Shooter/CurrentRPM", robot.shooter.getCurrentRPM());
        Fate.log("Shooter/TargetHoodPosition", robot.shooter.getTargetHoodPosition());
        Fate.log("Shooter/hoodPosition", robot.shooter.getCurrentHoodPosition());
    }
    private void updateTurret() {
        Fate.log("Turret/TargetAngle", robot.turret.getTargetAngleDegrees());
        Fate.log("Turret/CurrentAngle", robot.turret.getCurrentAngleDegrees());
        Fate.log("Turret/EstimatedAngle", robot.turret.getEstimatedAngle());
        Fate.log("Turret/Ready", robot.turret.isReady());
    }
    private void updateScorer() {
        Fate.log("Scorer/DistanceToHive", robot.scorer.getDistanceCm());
        Fate.log("Scorer/TurretAngle", robot.scorer.getTurretAngleDeg());
        Fate.log("Scorer/VirtualRobotPosition/x", robot.scorer.getVirtualPosition().elements[0]);
        Fate.log("Scorer/VirtualRobotPosition/y", robot.scorer.getVirtualPosition().elements[1]);
        Fate.log("Scorer/HivePosition/x", robot.scorer.getHivePosition().elements[0]);
        Fate.log("Scorer/HivePosition/y", robot.scorer.getHivePosition().elements[1]);
    }
    private void updateFollower() {
        Fate.log("Follower/Pose/x", robot.follower.pose().x());
        Fate.log("Follower/Pose/y", robot.follower.pose().y());
        Fate.log("Follower/Pose/heading", robot.follower.pose().heading());
    }
    private void updateCurrent() {
        Fate.log("Current/Voltage", robot.voltage);
        Fate.log("Current/Flywheel", robot.shooter.getCurrent());
        Fate.log("Current/Intake", robot.intake.getCurrent());
    }
    public void updateLoopTime() {
        Fate.log("Frequency (Hz)", robot.getFrequency());
    }

    public void update() {
        Fate.log("Alliance", robot.alliance.toString());

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
