package org.firstinspires.ftc.teamcode.utils.math;

import com.pedropathing.math.Vector;

public class SOTM {
    private static final int MAX_ITERATIONS = 5;

    private SOTM(){
        // private constructor to prevent the automatic construction from java
    }

    /**
     * Calculate the time of flight from the robot to the goal based on the distance and a formula
     * @param robotPosition The current position of the robot
     * @param goalPosition The position of the goal
     * @return The time of flight
     */
    private static double calculateTOF(Vector robotPosition, Vector goalPosition) {
        double distance = robotPosition.distance(goalPosition);
        return (distance * 0.005) + 0.5; // TODO: tune this calculation
    }

    /**
     * Calculate the virtual robot position based on the current robot position, velocity, and the goal position.
     * @param robotPosition The current position of the robot
     * @param robotVelocity The current velocity of the robot
     * @param goalPosition The position of the goal
     * @return The virtual position of the robot
     */
    public static Vector calculateVirtualRobotVector(Vector robotPosition, Vector robotVelocity, Vector goalPosition) {
        if (robotVelocity.magnitude() < 1)
            return robotPosition;

        Vector virtualRobotPosition = robotPosition;
        for (int i = 0; i <MAX_ITERATIONS; i++) {
            double TOF = calculateTOF(virtualRobotPosition, goalPosition);
            virtualRobotPosition = robotPosition.plus(robotVelocity.times(TOF));
        }
        return virtualRobotPosition;
    }
}
