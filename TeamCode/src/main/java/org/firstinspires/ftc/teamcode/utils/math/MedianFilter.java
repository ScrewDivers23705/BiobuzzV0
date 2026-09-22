package org.firstinspires.ftc.teamcode.utils.math;

import java.util.Arrays;

/**
 * A simple median filter implementation for smoothing noisy sensor data.
 * The filter remembers the N latest measurements and returns the median value.
 */
public class MedianFilter {
    private final double[] window;
    private final double[] buffer;
    private int index = 0;
    private boolean filled = false;
    private double lastMedian = 0.0;

    public MedianFilter(int size) {
        int windowSize = Math.max(1, size); // Minimum 1 size window
        this.window = new double[windowSize];
        this.buffer = new double[windowSize];
    }

    /**
     * Updates the filter with a new raw measurement and returns the median of the last N measurements.
     * @param rawMeasurement The new raw measurement to be added to the filter.
     * @return The median of the last N measurements, including the new raw measurement.
     */
    public double update(double rawMeasurement) {
        window[index] = rawMeasurement;
        index = (index + 1) % window.length;
        if (index == 0) {
            filled = true;
        }

        int count = filled ? window.length : index;

        // Copy the current window to the buffer
        System.arraycopy(window, 0, buffer, 0, count);
        Arrays.sort(buffer, 0, count);

        // Calculate median
        if (count % 2 == 1) {
            lastMedian = buffer[count / 2];
        } else {
            lastMedian = (buffer[(count / 2) - 1] + buffer[count / 2]) / 2.0;
        }
        return lastMedian;
    }

    /**
     * Gets the most recently calculated median value without pushing a new measurement.
     */
    public double getMedian() {
        return lastMedian;
    }

    public void reset() {
        index = 0;
        lastMedian = 0.0;
        filled = false;
    }
}
