package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.InterruptedBehavior;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utils.Alliance;
import org.firstinspires.ftc.teamcode.utils.LazyMotor;
import org.firstinspires.ftc.teamcode.utils.LazyServo;

import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.groups.Groups.parallel;

/**
 * Intake subsystem for the robot.
 * This class manages the intake motor and color sensors.
 */
@Configurable
public class Intake {

    /** Hardware components */
    private final LazyMotor intakeMotor;
    private final LazyServo rampServo;
    private final RevColorSensorV3 leftSensor;
    private final RevColorSensorV3 rightSensor;

    /** State variables */
    public static double intakePower = 1.0; // power for intake
    public static double disabledPower = 0.0; // power for disabled intake
    public static double reversePower = -1.0; // power for reverse intake
    public static double rampLowered = 0.0; // position for ramp lowered
    public static double rampRaised = 1.0; // position for ramp raised
    public static double detectionDistance = 8.0; // distance in cm for object detection
    private int loopCounter = 0; // loop counter for periodic checks
    private Alliance alliance = Alliance.BLUE;
    private boolean active = false; // whether the intake is active

    /** Sensor variables */
    public double red = 0, green = 0, blue = 0;
    public boolean leftDetected = false, rightDetected = false;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = new LazyMotor(hardwareMap, "intake");
        intakeMotor.setPowerTolerance(0.05); // since it's an intake we don't need to be precise with the power

        rampServo = new LazyServo(hardwareMap, "ramp");
        leftSensor = hardwareMap.get(RevColorSensorV3.class, "left_sensor");
        rightSensor = hardwareMap.get(RevColorSensorV3.class, "right_sensor");
    }

    private void set(double power) {
        intakeMotor.setPower(power);
    }
    private void intake() {
        active = true;
        set(intakePower);
    }
    private void stop() {
        active = false;
        set(disabledPower);
    }
    private void reverse() {
        active = true;
        set(reversePower);
    }
    private void raiseRamp() {
        rampServo.setPosition(rampRaised);
    }
    private void lowerRamp() {
        rampServo.setPosition(rampLowered);
    }
    public void setAlliance(Alliance alliance) {
        this.alliance = alliance;
    }
    /**
     * Checks if an object is detected by either of the color sensors.
     * @return true if an object is detected, false otherwise
     */
    private boolean isDetected(RevColorSensorV3 sensor) {
        return sensor.getDistance(DistanceUnit.CM) < detectionDistance;
    }
    private boolean isOpposingColor(RevColorSensorV3 sensor) {
        if (isDetected(sensor)) {
            if (sensor == leftSensor) {
                leftDetected = true;
            }
            else if (sensor == rightSensor) {
                rightDetected = true;
            }

            red = sensor.red();
            green = sensor.green();
            blue = sensor.blue();

            if (red > blue && green > blue) { // check if the color is yellow (red and green are high, blue is low)
                return false;
            }
            if (alliance == Alliance.BLUE) {
                return red > blue;
            }
            return blue > red;
        }
        return false;
    }

    public Command periodic() {
        return infinite(() -> {
            loopCounter++;
            if (loopCounter % 25 == 0 && active) { // only check every 25 loops to avoid high looptimes
                if (isOpposingColor(leftSensor) || isOpposingColor(rightSensor)) {
                    this.reverse();
                }
                else {
                    this.intake();
                }
            }
        })
        .requiring(intakeMotor)
        .setInterruptedBehavior(InterruptedBehavior.SUSPEND);
    }

    public boolean isActive() {
        return active;
    }

    public Command intakeCommand() {
        return instant(this::intake).requiring(intakeMotor);
    }
    public Command intakeFlowerCommand() {
        return parallel(instant(this::intake).requiring(intakeMotor), instant(this::lowerRamp).requiring(rampServo));
    }
    public Command reverseCommand() {
        return instant(this::reverse).requiring(intakeMotor);
    }
    public Command stopCommand() {
        return instant(this::stop).requiring(intakeMotor);
    }
    public Command raiseRampCommand() {
        return instant(this::raiseRamp).requiring(rampServo);
    }
    public Command lowerRampCommand() {
        return instant(this::lowerRamp).requiring(rampServo);
    }

    public double getCurrent() {
        return intakeMotor.getCurrent(CurrentUnit.AMPS);
    }
    public Command feedCommand() { // feed command for the shooteing sequence
        return Command.build()
                .requiring(intakeMotor)
                .setStart(this::intake)
                .setDone(() -> false)
                .setEnd(c -> stop());
    }
}
