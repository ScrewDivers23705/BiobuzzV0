package org.firstinspires.ftc.teamcode.OpModes.teleop.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.utils.LazyMotor;

@TeleOp
public class DriveTest extends OpMode {
    private LazyMotor left_back, left_front, right_back, right_front;

    @Override
    public void init() {
        left_back = new LazyMotor(hardwareMap, "left_back");
        left_front = new LazyMotor(hardwareMap, "left_front");
        right_back = new LazyMotor(hardwareMap, "right_back");
        right_front = new LazyMotor(hardwareMap, "right_front");

        left_back.setDirection(DcMotorSimple.Direction.REVERSE);
        left_front.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void loop() {
        mecanumDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
    }

    public void mecanumDrive(double forward, double strafe, double turn) {
        double denom = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(turn), 1);
        left_back.setPower((forward - strafe + turn) / denom);
        left_front.setPower((forward + strafe + turn) / denom);
        right_back.setPower((forward + strafe - turn) / denom);
        right_front.setPower((forward - strafe - turn) / denom);
    }
}
