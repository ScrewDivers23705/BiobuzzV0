package org.firstinspires.ftc.teamcode.utils;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.math.SOTM;

import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.loop;
import static com.pedropathing.ivy.groups.Groups.sequential;

@Configurable
public class Scorer {
    private final Robot robot;

    // ========================== Hive Constants ==========================
    private final static Vector RED_AUDIENCE_HIVE_POSITION = new Vector(58,55);
    private final static Vector RED_OPPOSITE_HIVE_POSITION = new Vector(58, 83);
    private final static Vector BLUE_AUDIENCE_HIVE_POSITION = new Vector(83, 55);
    private final static Vector BLUE_OPPOSITE_HIVE_POSITION = new Vector(83, 83);
    private Vector currentHivePosition = RED_AUDIENCE_HIVE_POSITION;
    private double distanceCm = 0.0;
    private double turretDeg = 0.0;
    private Vector virtualPos = new Vector(0,0);
    // ========================== Commands ==========================
    private Command shoot;
    public Scorer(Robot robot) {
        this.robot = robot;
    }

    /** this method calculates the distance and angle to the target hive and sets the shooter and turret for it.
     * @param alliance the alliance of the robot, used to determine which hive to target
     */
    public void periodic(Alliance alliance) {
        // get current robot state
        Pose pose = robot.follower.pose();
        Vector robotVelocity = new Vector(robot.follower.velocity().vx, robot.follower.velocity().vy);

        // determine current target hive position based on alliance and robot position
        if (alliance == Alliance.RED) { currentHivePosition = ( pose.y() >= 70.75) ? RED_OPPOSITE_HIVE_POSITION : RED_AUDIENCE_HIVE_POSITION;}
        else { currentHivePosition = ( pose.y() >= 70.75) ? BLUE_OPPOSITE_HIVE_POSITION : BLUE_AUDIENCE_HIVE_POSITION;}

        // calculate virtual robot position based on current robot state and target hive position
        Vector virtualRobotPosition = SOTM.calculateVirtualRobotVector(new Vector(pose.x(), pose.y()), robotVelocity, currentHivePosition);

        // calculate distance and angle to target hive position from virtual robot position
        double distanceToHive = virtualRobotPosition.distance(currentHivePosition) * 2.54; // convert from inches to centimeters
        double angleToHive = Math.toDegrees(Math.atan2(currentHivePosition.elements[1] - virtualRobotPosition.elements[1], currentHivePosition.elements[0] - virtualRobotPosition.elements[0]));

        // calculate turret angle to target hive position based on robot heading and angle to hive
        double robotHeading = Math.toDegrees(pose.heading());
        double turretAngle = angleToHive - robotHeading;

        double normalizedTurretAngle = normalizeAngle(turretAngle);

        robot.shooter.setDistance(distanceToHive);
        robot.turret.setTargetAngleDegrees(normalizedTurretAngle);

        // update class variables for telemetry
        distanceCm = distanceToHive;
        turretDeg = normalizedTurretAngle;
        virtualPos = virtualRobotPosition;
    }

    /** Normalizes an anglee in degrees in the range [-180, 180].
     * @param degrees the angle in degrees to normalize
     * @return the normalized angle in degrees
     */
    private double normalizeAngle(double degrees) {
        double angle = degrees % 360.0;
        if (angle > 180.0) {
            angle -= 360.0;
        } else if (angle <= -180.0) {
            angle += 360.0;
        }
        return angle;
    }

    private boolean ready() {
        return robot.shooter.isReady() && robot.turret.isReady();
    }
    /** creates a command that runs the whole shotting logic
     * @return a command that runs the shooting logic
     */
    public Command shootCommand() {
        if (shoot == null) {
            shoot = sequential(
                robot.shooter.spinUpCommand(), //start running flywheel
                loop(sequential(
                    waitUntil(this::ready), // wait until finished spinning up and turret is ready
                    robot.intake.feedCommand().until(() -> !ready())
                ))
            ).setPriority(1);
        }
        return shoot;
    }

    public boolean isReady() {
        return ready();
    }

    public double getDistanceCm() { return distanceCm; }
    public double getTurretAngleDeg() { return turretDeg; }
    public Vector getVirtualPosition() { return virtualPos; }
    public Vector getHivePosition() { return currentHivePosition; }


}
