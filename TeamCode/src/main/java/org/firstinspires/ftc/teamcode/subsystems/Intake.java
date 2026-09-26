package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.behaviors.InterruptedBehavior;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utils.Alliance;
import org.firstinspires.ftc.teamcode.utils.LazyMotor;

import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;

/**
 * Intake subsystem for the robot.
 * This class manages the intake motor and color sensors.
 */
@Configurable
public class Intake {

    /** Hardware components */
    private final LazyMotor intakeMotor;
    private final RevColorSensorV3 leftSensor;
    private final RevColorSensorV3 rightSensor;

    /** State variables */
    public static double intakePower = 1.0; // power for intake
    public static double disabledPower = 0.0; // power for disabled intake
    public static double reversePower = -1.0; // power for reverse intake

    private int loopCounter = 0; // loop counter for periodic checks

    private Alliance alliance = Alliance.BLUE;
    private boolean active = false; // whether the intake is active

    /** Sensor variables */
    public double Red = 0, Green = 0, Blue = 0;
    public boolean leftDetected = false, rightDetected = false;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = new LazyMotor(hardwareMap, "intake");
        intakeMotor.setPowerTolerance(0.05); // since it's an intake we don't need to be precise with the power

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

    public void setAlliance(Alliance alliance) {
        this.alliance = alliance;
    }
    /**
     * Checks if an object is detected by either of the color sensors.
     * @return true if an object is detected, false otherwise
     */
    private boolean isDetected(RevColorSensorV3 sensor) {
        return sensor.getDistance(DistanceUnit.CM) < 8.0;
    }
    private boolean isOpposingColor(RevColorSensorV3 sensor) {
        if (isDetected(sensor)) {
            if (sensor == leftSensor) {
                leftDetected = true;
            }
            else if (sensor == rightSensor) {
                rightDetected = true;
            }

            Red = sensor.red();
            Green = sensor.green();
            Blue = sensor.blue();

            if (Red > Blue && Green > Blue) { // check if the color is yellow (red and green are high, blue is low)
                return false;
            }
            if (alliance == Alliance.BLUE) {
                return Red > Blue;
            }
            return Blue > Red;
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
    public Command reverseCommand() {
        return instant(this::reverse).requiring(intakeMotor);
    }
    public Command stopCommand() {
        return instant(this::stop).requiring(intakeMotor);
    }

    public double getCurrent() {
        return intakeMotor.getCurrent(CurrentUnit.AMPS);
    }
}
