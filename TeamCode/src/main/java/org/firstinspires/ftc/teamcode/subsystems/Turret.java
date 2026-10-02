package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.InterruptedBehavior;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.utils.LazyServo;

import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;

@Configurable
public class Turret {
    private final LazyServo leftServo;
    private final LazyServo rightServo;

    public static double minAngleDegrees = -180; // minimum angle the turret can rotate to according to physical constraints of the robot
    public static double maxAngleDegrees = 180; // maximum angle the turret can rotate to according to physical constraints of the robot

    private double targetAngleDegrees = 0; // current target angle for the turret

    //================ for turret ready verification ===================
    public static double maxSpeedDegSec = 250; // maximum speed of the turret in degrees per second //todo check on actual hardware
    public static double readyThresholdDeg = 2; // threshold in degrees to consider the turret ready
    private double estimatedAngle = 0; // estimated angle of the turrret based on calculatiosn
    private long lastTime = 0; // last time the turret angle was updated

    public Turret(HardwareMap hardwareMap) {
        leftServo = new LazyServo(hardwareMap, "turret_left", minAngleDegrees, maxAngleDegrees);
        rightServo = new LazyServo(hardwareMap, "turret_right", minAngleDegrees, maxAngleDegrees);
        leftServo.setThreshold(0.0015);  // set threshold to 0.5 degrees instead of the default 1.8 degrees
        rightServo.setThreshold(0.0015); // set threshold to 0.5 degrees instead of the default 1.8 degrees

        setTargetAngleDegrees(0); // initialize the turret to the center position
        leftServo.setAngle(targetAngleDegrees);
        rightServo.setAngle(targetAngleDegrees);
    }

    public void setTargetAngleDegrees(double targetAngleDegrees) {
        // Clamp the target angle to the physical constraints of the turret
        this.targetAngleDegrees = Math.max(minAngleDegrees, Math.min(maxAngleDegrees, targetAngleDegrees));
    }
    public double getTargetAngleDegrees() {
        return targetAngleDegrees;
    }
    public double getCurrentAngleDegrees() {
        return leftServo.getAngle();
    }
    public boolean isReady() {
        return Math.abs(estimatedAngle - getTargetAngleDegrees()) < readyThresholdDeg;
    }
    public double getEstimatedAngle() {
        return estimatedAngle;
    }
    // ================ Commands ===================
    public Command setTargetAngleDegreesCommand(double targetAngleDegrees) {
        return instant(() -> setTargetAngleDegrees(targetAngleDegrees));
    }
    public Command periodic() {
        return infinite(() -> {
            leftServo.setAngle(targetAngleDegrees);
            rightServo.setAngle(targetAngleDegrees);

            // Update the estimated angle based on the target angle and the maximum speed of the turret
            long now = System.nanoTime();
            double dt = (lastTime == 0) ? 0 : (now - lastTime) / 1e9;
            double maxDeltaAngle = maxSpeedDegSec * dt;
            double angleError = targetAngleDegrees - estimatedAngle;
            estimatedAngle += Math.max(-maxDeltaAngle, Math.min(maxDeltaAngle, angleError));
            lastTime = now;
        })
        .requiring(leftServo, rightServo)
        .setInterruptedBehavior(InterruptedBehavior.SUSPEND);
    }
}
