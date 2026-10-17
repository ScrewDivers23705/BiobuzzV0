package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.InterruptedBehavior;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import org.firstinspires.ftc.teamcode.utils.LazyServo;

import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;

@Configurable
public class Turret {
    private final LazyServo leftServo;
    private final LazyServo rightServo;
    private final double halfServoRangeDegrees = (350 * 1.25) / 2; // how far the servos themselves can go in each direction

    public static double minAngleDegrees = -210; // minimum angle the turret can rotate to according to physical constraints of the robot
    public static double maxAngleDegrees = 210; // maximum angle the turret can rotate to according to physical constraints of the robot
    public static double leftOffsetDegrees = 0; // offset for the left servo to account for mechanical differences
    public static double rightOffsetDegrees = 0; // offset for the right servo to account for mechanical differences

    private double targetAngleDegrees = 0; // current target angle for the turret

    //================ for turret ready verification ===================
    public static double maxSpeedDegSec = 300; // maximum speed of the turret in degrees per second //todo check on actual hardware
    public static double readyThresholdDeg = 2; // threshold in degrees to consider the turret ready
    private double estimatedAngle = 0; // estimated angle of the turrret based on calculatiosn
    private long lastTime = 0; // last time the turret angle was updated

    public Turret(HardwareMap hardwareMap) {
        leftServo = new LazyServo(hardwareMap, "turret_left", -halfServoRangeDegrees, halfServoRangeDegrees);
        rightServo = new LazyServo(hardwareMap, "turret_right", -halfServoRangeDegrees, halfServoRangeDegrees);
        leftServo.setThreshold(0.001);  // set threshold to 0.44 degrees instead of the default 2.2 degrees
        rightServo.setThreshold(0.001); // set threshold to 0.44 degrees instead of the default 2.2 degrees
        leftServo.setPwmRange(new PwmControl.PwmRange(500, 2500)); // AXON MAX pwm range
        rightServo.setPwmRange(new PwmControl.PwmRange(500, 2500)); // AXON MAX pwm range

        setTargetAngleDegrees(0); // initialize the turret to the center position
        leftServo.setAngle(targetAngleDegrees + leftOffsetDegrees);
        rightServo.setAngle(targetAngleDegrees + rightOffsetDegrees);
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
            leftServo.setAngle(targetAngleDegrees + leftOffsetDegrees);
            rightServo.setAngle(targetAngleDegrees + rightOffsetDegrees);

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
