package org.firstinspires.ftc.teamcode.OpModes.teleop.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import gay.zharel.fateweaver.flight.FateFtcTelemetry;
import gay.zharel.fateweaver.telemetry.Fate;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@Config
@TeleOp(name = "Shooter Test", group = "test")
public class ShooterTest extends OpMode {
    private Shooter shooter;
    public static double RPM = 0;
    public static double hoodPosition = 0;

    public void init() {
        shooter = new Shooter(hardwareMap);

        telemetry = new MultipleTelemetry(
                super.telemetry,
                FtcDashboard.getInstance().getTelemetry(),
                FateFtcTelemetry.INSTANCE
        );

        Scheduler.reset();
    }
    public void start() {
        shooter.setManualControl(true);
        shooter.spinUpCommand().schedule();
        shooter.periodic().schedule();
    }

    public void loop() {
        Scheduler.execute();
        if (gamepad1.dpad_up) { shooter.spinUpCommand().schedule(); }
        if (gamepad1.dpad_down) { shooter.spinDownCommand().schedule(); }

        if (gamepad1.rightTriggerWasPressed()) {
            shooter.setTargetRPM(RPM);
            shooter.setTargetHoodPosition(hoodPosition);
        }

        telemetry.addData("RPM: ", shooter.getCurrentRPM());
        telemetry.addData("target RPM: ", shooter.getTargetRPM());
        telemetry.addData("hood position: ", shooter.getTargetHoodPosition());
        telemetry.addData("Current (Amps): ", shooter.getCurrent());
        telemetry.addData("Current Voltage (V): ", shooter.getCurrentVoltage());

        Fate.log("RPM", shooter.getCurrentRPM());
        Fate.log("target RPM", shooter.getTargetRPM());
        Fate.log("hood position", shooter.getTargetHoodPosition());
        Fate.log("Current (Amps)", shooter.getCurrent());
        Fate.log("Current Voltage (V)", shooter.getCurrentVoltage());

        telemetry.update();
    }

}
