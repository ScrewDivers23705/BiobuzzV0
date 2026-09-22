package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.utils.math.LowPassFilter;

/**
 * A caching wrapper around {@link DcMotorEx} that skips useless hardware writes.
 * Set power/velocity/mode every loop as normal writes are only sent to the motor
 * when the value changes beyond a preset tolerance.
 *
 * <pre>{@code
 * LazyMotor shooter = new LazyMotor(hardwareMap, "shooterMotor");
 * shooter.setVelocityRPM(3000);
 * if (shooter.isStalled(4.0, 500)) shooter.stop();
 * }</pre>
 *
 */
public class LazyMotor implements DcMotorEx {

    private final DcMotorEx motor;

    // Cached variables for writes
    private double power = 0.0;
    private double targetVelocity = 0.0;
    private int targetPosition = 0;
    private RunMode mode;
    private ZeroPowerBehavior zeroPowerBehavior;
    private Direction direction;

    // Default thresholds for determining if a write is necessary
    private double powerTolerance = 0.001;
    private double velocityTolerance = 0.1;

    // Cached variables for current reads
    private double cachedCurrentMA = 0.0;
    private long lastCurrentReadTimestamp = 0;
    private int currentCacheIntervalMS = 50;

    // Values for stall detection
    private final LowPassFilter currentFilter = new LowPassFilter(0.3);
    private long stallStartTime = 0;

    // DcMotorEx constructor
    public LazyMotor(DcMotorEx motor) {
        this.motor = motor;

        // Set initial cache values from hardware
        refreshCache();

        this.lastCurrentReadTimestamp = 0;
        this.cachedCurrentMA = 0.0;
        this.targetVelocity = 0.0;

        // Stop motor on creation
        stop();
    }
    // HardwareMap and motor name constructor
    public LazyMotor(HardwareMap hardwareMap, String name) {
        this(hardwareMap.get(DcMotorEx.class, name));
    }

    // Threshold setters & getters
    public void setPowerTolerance(double powerTolerance) {
        this.powerTolerance = powerTolerance;
    }

    public void setVelocityTolerance(double velocityTolerance) {
        this.velocityTolerance = velocityTolerance;
    }

    public double getPowerTolerance() {
        return this.powerTolerance;
    }

    public double getVelocityTolerance() {
        return this.velocityTolerance;
    }

    public void setCurrentCacheIntervalMS(int intervalMS) {
        this.currentCacheIntervalMS = intervalMS;
    }

    public int getCurrentCacheIntervalMS() {
        return this.currentCacheIntervalMS;
    }

    // --- Lazy writes ---

    @Override
    public void setPower(double power) {
        setPower(power,false);
    }
    /**
        * Sets the power of the motor, but only writes to the hardware if the new power is different from the cached value by more than the powerTolerance.
        * @param power The new power to set the motor [-1.0, 1.0].
        * @param force If true the power will set no matter the tolerance.
     */

    public void setPower(double power,boolean force) {
        double clampedPower = Math.max(-1.0, Math.min(1.0, power));
        if (force || (clampedPower == 0.0 && this.power != 0.0) || Math.abs(clampedPower - this.power) > powerTolerance) {
            motor.setPower(clampedPower);
            this.power = clampedPower;
        }
    }

    /**
     * Sets motor power scaled to compensate for battery voltage drop.
     * @param power Target baseline power [-1.0, 1.0].
     * @param currentVoltage Current voltage reading from the robot's VoltageSensor.
     * @param nominalVoltage Baseline voltage (typically 12.0V).
     */
    public void setPowerCompensated(double power, double currentVoltage, double nominalVoltage) {
        if (currentVoltage <= 0) {
            setPower(power);
            return;
        }
        double compensatedPower = power * (nominalVoltage / currentVoltage);
        setPower(compensatedPower);
    }

    @Override
    public void setVelocity(double velocity) {
        setVelocity(velocity,false);
    }
    /**
        * Sets the velocity of the motor, but only writes to the hardware if the new velocity is different from the cached value by more than the velocityTolerance.
        * @param velocity The new velocity to set the motor in ticks/second.
        * @param force If true the velocity will set no matter the tolerance.
     */
    public void setVelocity(double velocity,boolean force) {
        if (force || (velocity == 0.0 && this.targetVelocity != 0.0) || Math.abs(velocity - this.targetVelocity) > velocityTolerance) {
            this.targetVelocity = velocity;
            motor.setVelocity(this.targetVelocity);
        }
    }

    @Override
    public void setVelocity(double angularRate, AngleUnit unit) {
        double ticksPerRev = motor.getMotorType().getTicksPerRev();
        if (ticksPerRev == 0) {
            motor.setVelocity(angularRate, unit);
            return;
        }
        double revolutionsPerSecond = unit.toDegrees(angularRate) / 360.0;
        double ticksPerSecond = revolutionsPerSecond * ticksPerRev;
        setVelocity(ticksPerSecond, false);
    }

    public void setVelocityRPM(double rpm) {
        setVelocityRPM(rpm,false);
    }
    /**
     * Sets the velocity of the motor in Revolutions Per Minute (RPM).
     * @param rpm The target speed in RPM.
     * @param force If true the velocity will set no matter the tolerance.
     */
    public void setVelocityRPM(double rpm, boolean force) {
        double ticksPerRev = motor.getMotorType().getTicksPerRev();
        if (ticksPerRev == 0) return;
        double ticksPerSecond = (rpm * ticksPerRev) / 60.0;
        setVelocity(ticksPerSecond, force);
    }

    /**
     * Gets the current velocity of the motor in Revolutions Per Minute (RPM).
     * @return The current speed in RPM.
     */
    public double getVelocityRPM() {
        double ticksPerRev = motor.getMotorType().getTicksPerRev();
        if (ticksPerRev == 0) return 0;
        return (getVelocity() * 60.0) / ticksPerRev;
    }

    /**
     * Sets the target position of the motor, but only writes to the hardware if the new position is different from the cached value.
     * @param position The new target position to set the motor.
     */
    @Override
    public void setTargetPosition(int position) {
        if (position != this.targetPosition) {
            this.targetPosition = position;
            motor.setTargetPosition(position);
        }
    }

    /**
     * Immediately stops the motor by forcing power to zero and zeroing target velocity.
     */
    public void stop() {
        this.targetVelocity = 0.0;
        setPower(0.0, true);
    }

    // --- Motor configuration methods ---
    @Override
    public void setMode(RunMode mode) {
        if (mode != this.mode) {
            this.mode = mode;
            motor.setMode(mode);
            if (mode == RunMode.STOP_AND_RESET_ENCODER) {
                this.targetPosition = 0;
                this.power = 0.0;
                this.targetVelocity = 0.0;
            }
        }
    }

    @Override
    public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {
        if (zeroPowerBehavior != this.zeroPowerBehavior) {
            this.zeroPowerBehavior = zeroPowerBehavior;
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    @Override
    public void setDirection(Direction direction) {
        if (direction != this.direction) {
            this.direction = direction;
            motor.setDirection(direction);
        }
    }

    public void resetEncoder() {
        RunMode previousMode = getMode();
        setMode(RunMode.STOP_AND_RESET_ENCODER);
        setMode(previousMode);
    }

    public void setInverted(boolean inverted) {
        setDirection(inverted ? Direction.REVERSE : Direction.FORWARD);
    }

    public boolean isInverted() {
        return getDirection() == Direction.REVERSE;
    }

    /**
     * Refreshes cache.
     */
    public void refreshCache() {
        this.mode = motor.getMode();
        this.zeroPowerBehavior = motor.getZeroPowerBehavior();
        this.direction = motor.getDirection();
        this.power = motor.getPower();
        this.targetPosition = motor.getTargetPosition();
        this.targetVelocity = 0.0;
    }

    // --- Cached getters ---

    @Override
    public double getPower(){
        return this.power;
    }
    public double getTargetVelocity(){
        return this.targetVelocity;
    }
    @Override
    public int getTargetPosition(){
        return this.targetPosition;
    }
    @Override
    public RunMode getMode(){
        return this.mode;
    }
    @Override
    public ZeroPowerBehavior getZeroPowerBehavior(){
        return this.zeroPowerBehavior;
    }
    @Override
    public Direction getDirection(){
        return this.direction;
    }

    // --- Direct hardware access ---

    @Override public int getCurrentPosition() { return motor.getCurrentPosition(); }
    @Override public double getVelocity() { return motor.getVelocity(); }
    @Override public boolean isBusy() { return motor.isBusy(); }

    @Override public void setMotorEnable() { motor.setMotorEnable(); }
    @Override public void setMotorDisable() { motor.setMotorDisable(); }
    @Override public boolean isMotorEnabled() { return motor.isMotorEnabled(); }

    @Override public Manufacturer getManufacturer() { return motor.getManufacturer(); }
    @Override public String getDeviceName() { return motor.getDeviceName(); }
    @Override public String getConnectionInfo() { return motor.getConnectionInfo(); }
    @Override public int getVersion() { return motor.getVersion(); }
    @Override public void resetDeviceConfigurationForOpMode() { motor.resetDeviceConfigurationForOpMode(); refreshCache(); }
    @Override public void close() { motor.close(); }

    // --- Unit & Velocity Overloads ---
    @Override public double getVelocity(AngleUnit unit) { return motor.getVelocity(unit); }

    // --- PID & PIDF Methods ---
    @Override public void setPIDCoefficients(RunMode mode, PIDCoefficients pidCoefficients) { motor.setPIDCoefficients(mode, pidCoefficients); }
    @Override public void setPIDFCoefficients(RunMode mode, PIDFCoefficients pidfCoefficients) { motor.setPIDFCoefficients(mode, pidfCoefficients); }
    @Override public PIDCoefficients getPIDCoefficients(RunMode mode) { return motor.getPIDCoefficients(mode); }
    @Override public PIDFCoefficients getPIDFCoefficients(RunMode mode) { return motor.getPIDFCoefficients(mode); }
    @Override public void setVelocityPIDFCoefficients(double p, double i, double d, double f) { motor.setVelocityPIDFCoefficients(p, i, d, f); }
    @Override public void setPositionPIDFCoefficients(double p) { motor.setPositionPIDFCoefficients(p); }

    // --- Target Position Tolerance ---
    @Override public void setTargetPositionTolerance(int tolerance) { motor.setTargetPositionTolerance(tolerance); }
    @Override public int getTargetPositionTolerance() { return motor.getTargetPositionTolerance(); }

    // --- Current ---
    @Override
    public double getCurrent(CurrentUnit unit) {
        long now = System.currentTimeMillis();
        if (now - lastCurrentReadTimestamp > currentCacheIntervalMS) {
            double rawCurrentMA = motor.getCurrent(CurrentUnit.MILLIAMPS);

            if (lastCurrentReadTimestamp == 0) {
                currentFilter.reset();
            }
            cachedCurrentMA = currentFilter.update(rawCurrentMA);
            lastCurrentReadTimestamp = now;
        }
        return unit == CurrentUnit.AMPS ? cachedCurrentMA / 1000.0 : cachedCurrentMA;
    }
    @Override public double getCurrentAlert(CurrentUnit unit) { return motor.getCurrentAlert(unit); }
    @Override public void setCurrentAlert(double current, CurrentUnit unit) { motor.setCurrentAlert(current, unit); }
    @Override public boolean isOverCurrent() { return motor.isOverCurrent(); }

    // --- Motor Type, Controller & Port Info ---
    @Override public MotorConfigurationType getMotorType() { return motor.getMotorType(); }
    @Override public void setMotorType(MotorConfigurationType motorType) { motor.setMotorType(motorType); }
    @Override public DcMotorController getController() { return motor.getController(); }
    @Override public int getPortNumber() { return motor.getPortNumber(); }

    // --- Power Float Methods ---
    @Override public void setPowerFloat() { motor.setPowerFloat(); this.power = 0; }
    @Override public boolean getPowerFloat() { return motor.getPowerFloat(); }

    /**
     * Checks if the motor current exceeds a threshold for a sustained duration.
     * @param currentThresholdAmps The current threshold in Amps.
     * @param requiredDurationMS How long the current must remain above the threshold to be considered a stall.
     * @return True if the motor is currently stalled.
     */
    public boolean isStalled(double currentThresholdAmps, long requiredDurationMS) {
        double currentAmps = getCurrent(CurrentUnit.AMPS);
        if (currentAmps >= currentThresholdAmps) {
            if (stallStartTime == 0) {
                stallStartTime = System.currentTimeMillis();
            }
            return (System.currentTimeMillis() - stallStartTime) >= requiredDurationMS;
        } else {
            stallStartTime = 0;
            return false;
        }
    }
    public void resetCurrentFilter() {
        currentFilter.reset();
        lastCurrentReadTimestamp = 0;
    }
}
