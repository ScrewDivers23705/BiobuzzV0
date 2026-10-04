package org.firstinspires.ftc.teamcode.subsystems.vision;

import android.util.Size;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.utils.Alliance;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.List;
import java.util.concurrent.TimeUnit;
// TODO: add to TeamCode/src/main/res/xml/teamwebcamcalibrations.xml the camera calibration for the webcam
@Configurable
public class AprilTagVision {

    public static final class Target {
        public final String name;
        public final double rangeCm, bearingDeg, elevationDeg, rollDeg;
        public final int percentClusterFound;

        Target(String name, double rangeCm, double bearingDeg, double elevationDeg, double rollDeg, int percentClusterFound) {
            this.name = name;
            this.rangeCm = rangeCm;
            this.bearingDeg = bearingDeg;
            this.elevationDeg = elevationDeg;
            this.rollDeg = rollDeg;
            this.percentClusterFound = percentClusterFound;
        }
    }

    // ========== Vision System ==========
    private final AprilTagProcessor aprilTag;
    private final VisionPortal visionPortal;

    // ========== Configurables ==========
    private static final Position CAMERA_POSITION = new Position(DistanceUnit.CM,
            0, 0, 0,0); // TODO: check actual camera position relative to robot center
    private static final YawPitchRollAngles CAMERA_ORIENTATION = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, 0, 0, 0); // TODO: check actual camera orientation relative to robot center

    public static int CAMERA_WIDTH = 640; // Camera width in pixels
    public static int CAMERA_HEIGHT = 480; // Camera height in pixels
    public static int minPercentClusterFound = 50; // Minimum percent of cluster visable for it to be acutally like found and good
    public static double DECIMATION = 2.0; // Decimation factor for AprilTag detection (higher = more fps but shorter)
    public static boolean LIVE_VIEW = true; // Whether to show the camera feed on the driver station
    public static boolean MANUAL_EXPOSURE = false; // Whether to use manual exposure or auto exposure for the camera
    public static long EXPOSURE_MS = 6; // Exposure time in milliseconds for manual exposure
    public static int GAIN = 100; // Gain for manual exposure (0-255)

    private boolean settingsApplied = false; // Whether the settings have been applied to the camera
    private Target latest = null; // The latest target detected

    public AprilTagVision(HardwareMap hardwareMap) {
        aprilTag = new AprilTagProcessor.Builder()
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .setDrawTagID(true) // TODO: remove when done testing
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .build();
        aprilTag.setDecimation((float) DECIMATION);

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(CAMERA_WIDTH, CAMERA_HEIGHT))
                .enableLiveView(LIVE_VIEW)
                .addProcessor(aprilTag)
                .build();
    }

    /** called once per loop, updates the latest target found */
    public void periodic(Alliance alliance) {
        applyCameraSettings();
        latest = findTarget(alliance);
    }

    /** returns the latest target found, or null if none found */
    public Target getTarget() {
        return latest;
    }

    public VisionPortal.CameraState getCameraState() {
        return visionPortal.getCameraState();
    }

    public void close() {
        visionPortal.close(); // to help cpu
    }

    private Target findTarget(Alliance alliance) {
        char allianceChar = (alliance == Alliance.RED) ? 'R' : 'B';
        Target best = null;

        List<AprilTagDetection> detections = aprilTag.getDetections();
        for (AprilTagDetection detection : detections) {
            if (!(detection instanceof AprilTagClusterDetection)) continue;

            AprilTagClusterDetection clusterDetection = (AprilTagClusterDetection) detection;
            String name = clusterDetection.metadata.name;
            if (name == null || name.isEmpty() || Character.toUpperCase(name.charAt(0)) != allianceChar) continue; // ignore tags that aren't for our alliance
            if (Math.abs(clusterDetection.ftcPose.roll) >= 90) continue; // ignore tags that aren't looking up
            if (clusterDetection.percentClusterFound < minPercentClusterFound) continue; // ignore tags that aren't mostly visible

            Target target = new Target(name, clusterDetection.ftcPose.range, clusterDetection.ftcPose.bearing,
                    clusterDetection.ftcPose.elevation, clusterDetection.ftcPose.roll, clusterDetection.percentClusterFound);
            if (best == null || target.percentClusterFound > best.percentClusterFound || (target.percentClusterFound == best.percentClusterFound && target.rangeCm < best.rangeCm)) {
                best = target;
            }
        }
        return best;
    }

    public void applyCameraSettings() {
        if (settingsApplied || !MANUAL_EXPOSURE) return;
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) return;

        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
        }
        exposureControl.setExposure(EXPOSURE_MS, TimeUnit.MILLISECONDS);
        visionPortal.getCameraControl(GainControl.class).setGain(GAIN);
        settingsApplied = true;

    }

}
