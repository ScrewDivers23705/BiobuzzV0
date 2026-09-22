package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.*;

/**
 * A wrapper class for the Servo class that only writes to the hardware when the position changes by a certain threshold.
 * This can help reduce the number of writes to the hardware and improve performance.
 * <pre>
 *     LazyServo lazyServo = new LazyServo(hardwareMap, "servo")
 *         .setThreshold(0.01);
 *
 *     lazyServo.setThreshold(0.005);
 *     lazyServo.setAngle(90);
 * </pre>
 */
public class LazyServo implements Servo {
    private Servo servo;
    private double position = Double.NaN;
    private double threshold = 0.005; // Default threshold value
    private double min = 0.0; // Minimum range value
    private double max = 1.0; // Maximum range value

    // --- Constructors ---

    public LazyServo(Servo servo) {
        this(servo, 0.0, 1.0);
    }

    /**
     * Creates a LazyServo that wraps around a Servo object.
     * @param servo The Servo object to wrap around.
     * @param min The minimum range value for the servo.
     * @param max The maximum range value for the servo.
     */
    public LazyServo(Servo servo, double min, double max) {
        this.servo = servo;
        this.position = Double.NaN;
        this.min = min;
        this.max = max;
    }
    /**
     * Creates a LazyServo that wraps around a Servo object.
     * @param hardwareMap The HardwareMap to use.
     * @param servoName The name of the servo in the HardwareMap.
     * @param min The minimum range value for the servo.
     * @param max The maximum range value for the servo.
     */
    public LazyServo(HardwareMap hardwareMap, String servoName, double min, double max) {
        this(hardwareMap.get(Servo.class, servoName), min, max);
    }

    /**
     * Creates a LazyServo that wraps around a Servo object.
     * @param hardwareMap The HardwareMap to use.
     * @param servoName The name of the servo in the HardwareMap.
     */
    public LazyServo(HardwareMap hardwareMap, String servoName) {
        this(hardwareMap.get(Servo.class, servoName), 0.0, 1.0);
    }

    /**
     *  Sets the change threshold to trigger a hardware write.
     * @param threshold The new threshold value.
     */
    public LazyServo setThreshold(double threshold) {
        this.threshold = threshold;
        return this;
    }

    @Override
    public void setPosition(double position) {
        setPosition(position, false);
    }
    /**
     * Sets the position of the servo, but only if the change in position is greater than the threshold.
     * @param position The new position of the servo.
     * @param force If true the position will be set no matter the threshold.
     */
    public void setPosition(double position, boolean force) {
        if (force || Double.isNaN(this.position) || Math.abs(position - this.position) >= threshold) {
            this.position = position;
            servo.setPosition(position);
        }
    }

    @Override
    public double getPosition() {
        return Double.isNaN(position) ? servo.getPosition() : position;
    }

    /**
     * Sets the servo position using real angles or custom units (e.g. 0 to 270).
     * @param angle The target angle between min and max.
     */
    public void setAngle(double angle) {
        double normalizedPosition = (angle - min) / (max - min);
        double clampedPosition = Math.max(0.0, Math.min(1.0, normalizedPosition));
        setPosition(clampedPosition);
    }

    /**
     * Gets the current position mapped back into real angle units.
     */
    public double getAngle() {
        return getPosition() * (max - min) + min;
    }

    /**
     * Configures extended PWM pulse widths for gobilda smart servos or genrallly ones that need them.
     * for example: setPwmRange(new PwmControl.PwmRange(500, 2500))
     */
    public LazyServo setPwmRange(PwmControl.PwmRange pwmRange) {
        if (servo.getController() instanceof ServoControllerEx) {
            ((ServoControllerEx) servo.getController()).setServoPwmRange(servo.getPortNumber(), pwmRange);
        }
        return this;
    }

    @Override
    public void scaleRange(double min, double max) {
        servo.scaleRange(min, max);
    }

    @Override
    public void setDirection(Direction direction) {
        servo.setDirection(direction);
    }

    @Override
    public Direction getDirection() {
        return servo.getDirection();
    }

    @Override
    public void close(){
        servo.close();
    }

    @Override public ServoController getController() {return servo.getController(); }

    @Override public int getPortNumber() { return servo.getPortNumber(); }

    @Override public Manufacturer getManufacturer() { return servo.getManufacturer(); }

    @Override public String getDeviceName() { return servo.getDeviceName(); }

    @Override public String getConnectionInfo() { return servo.getConnectionInfo(); }

    @Override public int getVersion() { return servo.getVersion(); }

    @Override public void resetDeviceConfigurationForOpMode() { servo.resetDeviceConfigurationForOpMode(); }


}
