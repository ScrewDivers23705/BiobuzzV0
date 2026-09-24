package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.utils.LazyMotor;
import org.firstinspires.ftc.teamcode.utils.LazyServo;
import org.firstinspires.ftc.teamcode.utils.math.LookUpTable;

@Configurable
public class Shooter {

    private final LazyMotor flywheel;
    private final LazyServo hood;
    private LookUpTable lookUpTable;

    public static double kP = 0, kV = 0, kS = 0; // TODO tune them according to flywheel
    public static double tolerance = 30; // RPM tolerance for the flywheel to be considered "ready"

    // State variables for the shooter
    private boolean active = false;
    private double targetRPM = 0;
    private double currentRPM = 0;
    private double targetHoodPosition = 0;

    private double currentVoltage = 12; // current battery voltage, used for feedforward control

    public Shooter(HardwareMap hardwareMap) {
        flywheel = new LazyMotor(hardwareMap, "flywheel"); // initialize the flywheel motor
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // set to run without encoder for our own PIDFs control
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel.setPowerTolerance(0.001); // since it's a flywheel we want to be able to adjust the power very precisely

        hood = new LazyServo(hardwareMap, "hood"); // initialize the hood servo

        setupLookUpTable(); // setup the lookup table for distance to RPM and hood position
    }

    public void periodic(double distance) {
        // Update the target RPM and hood position based on the distance using the lookup table
        if (lookUpTable != null) {
            double[] outputs = lookUpTable.get(distance);
            targetRPM = outputs[0];
            targetHoodPosition = outputs[1];
        }

        currentRPM = flywheel.getVelocityRPM();

        if (active && targetRPM > 0) {
            flywheel.setPowerCompensated(calculateFlywheelPower(), currentVoltage, 12.0); // TODO change the nominal voltage to whatever it was tuned at.
            hood.setPosition(targetHoodPosition);
        }
        else {
            flywheel.setPower(0.0);
        }
    }
    public double calculateFlywheelPower() {
        // Calculate the power needed to reach the target RPM using a simple feedforward control
        double error = targetRPM - currentRPM;
        return (kP * error) + (kV * targetRPM) + (kS * Math.signum(targetRPM)); // standard feedforward control calculation
    }

    public boolean isReady(){
        // Check if the flywheel is within a certain tolerance of the target RPM
        return active && (currentRPM > 0) && (Math.abs(targetRPM - currentRPM) <= tolerance);
    }

    public void setupLookUpTable() {
        this.lookUpTable = new LookUpTable(2); // 2 outputs: RPM and hood position
        // Add entries to the lookup table (distance in cm, RPM, hood position)
        //lookUpTable.add(100, 3000, 0.2); // 100cm: 3000 RPM, hood position 0.2 for exmaple
        //TODO find acutal points for the lookuptable
    }

    // Getters and setters for the shooter state variables
    public boolean isActive() { return active; }
    public void setActive(boolean active) { this.active = active; }
    public double getTargetRPM() { return targetRPM; }
    public void setTargetRPM(double targetRPM) { this.targetRPM = targetRPM; }
    public double getCurrentRPM() { return currentRPM; }
    public double getTargetHoodPosition() { return targetHoodPosition; }
    public double getCurrentHoodPosition() { return hood.getPosition(); }
    public void setTargetHoodPosition(double targetHoodPosition) { this.targetHoodPosition = targetHoodPosition; }
    public double getCurrentVoltage() { return currentVoltage; }
    public void setCurrentVoltage(double currentVoltage) { this.currentVoltage = currentVoltage; }

}
