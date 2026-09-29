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

    public Turret(HardwareMap hardwareMap) {
        leftServo = new LazyServo(hardwareMap, "turret_left", minAngleDegrees, maxAngleDegrees);
        rightServo = new LazyServo(hardwareMap, "turret_right", minAngleDegrees, maxAngleDegrees);
        leftServo.setThreshold(0.0015);  // set threshold to 0.5 degrees instead of the default 1.8 degrees
        rightServo.setThreshold(0.0015); // set threshold to 0.5 degrees instead of the default 1.8 degrees

        setTargetAngleDegrees(0); // initialize the turret to the center position
        leftServo.setPosition(targetAngleDegrees);
        rightServo.setPosition(targetAngleDegrees);
    }

    public void setTargetAngleDegrees(double targetAngleDegrees) {
        // Clamp the target angle to the physical constraints of the turret
        this.targetAngleDegrees = Math.max(minAngleDegrees, Math.min(maxAngleDegrees, targetAngleDegrees));
    }
    public double getTargetAngleDegrees() {
        return targetAngleDegrees;
    }
    public double getCurrentAngleDegrees() {
        return leftServo.getPosition();
    }
    // ================ Commands ===================
    public Command setTargetAngleDegreesCommand(double targetAngleDegrees) {
        return instant(() -> setTargetAngleDegrees(targetAngleDegrees));
    }
    public Command periodic() {
        return infinite(() -> {
            leftServo.setAngle(targetAngleDegrees);
            rightServo.setAngle(targetAngleDegrees);
        })
        .requiring(leftServo, rightServo)
        .setInterruptedBehavior(InterruptedBehavior.SUSPEND);
    }
}
