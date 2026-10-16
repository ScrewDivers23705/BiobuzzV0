package org.firstinspires.ftc.teamcode.OpModes.teleop;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.vision.AprilTagVision;
import org.firstinspires.ftc.teamcode.utils.Alliance;

@TeleOp
public class visionTest extends OpMode {
    AprilTagVision vision;

    @Override
    public void init() {
        vision = new AprilTagVision(hardwareMap);
        vision.applyCameraSettings();
    }

    @Override
    public void loop() {
        vision.periodic(Alliance.BLUE);
        if (vision.getTarget() != null) {
            telemetry.addData("Name: ", vision.getTarget().name);
            telemetry.addData("bearing: ", vision.getTarget().bearingDeg);
            telemetry.addData("elevation: ", vision.getTarget().elevationDeg);
            telemetry.addData("Percent cluster: ", vision.getTarget().percentClusterFound);
            telemetry.addData("distance: ", vision.getTarget().rangeCm);
            telemetry.addData("roll deg:", vision.getTarget().rollDeg);
            telemetry.addLine("ftc pose:");
            telemetry.addData(" cluster ftcpose X: ", vision.getTarget().clusterDetection.ftcPose.x);
            telemetry.addData(" cluster ftcpose Y: ", vision.getTarget().clusterDetection.ftcPose.y);
            telemetry.addData(" cluster ftcpose Z: ", vision.getTarget().clusterDetection.ftcPose.z);
            telemetry.addLine("robot pose:");
            telemetry.addData(" robot ftcpose X: ", vision.getTarget().clusterDetection.robotPose.getPosition().x);
            telemetry.addData(" robot ftcpose Y: ", vision.getTarget().clusterDetection.robotPose.getPosition().y);
            telemetry.addData(" robot ftcpose Z: ", vision.getTarget().clusterDetection.robotPose.getPosition().z);
        }
        else {
            telemetry.addData("Camerastate", vision.getCameraState());
            telemetry.addLine("No target found");
        }
        telemetry.update();
    }

}
