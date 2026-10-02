package org.firstinspires.ftc.teamcode.OpModes.teleop;

import com.pedropathing.follower.ManualDrive;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.OpModes.CommandOpMode;

@TeleOp(name = "Test Teleop", group = "Test")
public class TestTeleop extends CommandOpMode {

    @Override
    public void loop() {
        commandLoop(() -> {

            // manual drive using pedro
            ManualDrive.driveOrHold(robot.follower, -gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x); // manually drive the robot using pedro

            // commands to intake
            if (gamepad1.leftBumperWasPressed()) { robot.intake.intakeCommand().schedule(); }
            if (gamepad1.leftBumperWasReleased()) { robot.intake.stopCommand().schedule(); }

            // commands to shoot
            if (gamepad1.rightTriggerWasPressed()) { robot.scorer.shootCommand().schedule(); }
            if (gamepad1.rightTriggerWasReleased()) { robot.scorer.shootCommand().cancel(); }
            if (gamepad1.triangleWasPressed()) { robot.shooter.spinDownCommand().schedule(); }
        });
    }
}
