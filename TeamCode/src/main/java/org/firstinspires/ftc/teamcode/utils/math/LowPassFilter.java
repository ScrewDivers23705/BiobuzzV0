package org.firstinspires.ftc.teamcode.utils.math;
/**
 * A simple low-pass filter implementation for smoothing noisy sensor data.
 * The filter uses an exponential moving average.
 */
public class LowPassFilter {
    private double alpha; // Smoothing weight between 0 and 1, where 0 means max smoothing and 1 means no smoothing
    private double currentEstimate = Double.NaN;

    public  LowPassFilter(double alpha) {
        this.alpha = alpha;
    }

    public void setAlpha(double alpha) {
        this.alpha = alpha;
    }

    public double update(double rawMeasurement) {
        if (Double.isNaN(currentEstimate)) {
            currentEstimate = rawMeasurement;
        }
        else {
            currentEstimate = (alpha * rawMeasurement) + ((1 - alpha) * currentEstimate);
        }
        return currentEstimate;
    }

    public void reset(){
        currentEstimate = Double.NaN;
    }
}
