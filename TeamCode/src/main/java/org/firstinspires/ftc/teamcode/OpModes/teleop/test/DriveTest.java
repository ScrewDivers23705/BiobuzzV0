package org.firstinspires.ftc.teamcode.OpModes.teleop.test;

import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.utils.LazyMotor;

@TeleOp
public class DriveTest extends OpMode {
    private LazyMotor left_back, left_front, right_back, right_front;
    private final Timer loop = new Timer();
    public double loops = 0, lastLoop = 0, loopTime = 1;

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


        if (lastLoop == 0) {
            lastLoop = loop.milliseconds();
        }


        if (loops >= 25) { // only calculate looptime after 25 loops to avoid initial startup lag
            double curTime = loop.milliseconds(); // get the current time in milliseconds
            loopTime = (curTime - lastLoop) / loops; // calculate the average looptime in milliseconds
            lastLoop = curTime; // set lastLoop to the current time for the next calculation
            loops = 0; // reset loops for the next calculation
        }
        loops++;
        telemetry.addData("looptime (ms): ", loopTime);
        telemetry.update();

    }

    public void mecanumDrive(double forward, double strafe, double turn) {
        double denom = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(turn), 1);
        left_back.setPower((forward - strafe + turn) / denom);
        left_front.setPower((forward + strafe + turn) / denom);
        right_back.setPower((forward + strafe - turn) / denom);
        right_front.setPower((forward - strafe - turn) / denom);
    }
}
