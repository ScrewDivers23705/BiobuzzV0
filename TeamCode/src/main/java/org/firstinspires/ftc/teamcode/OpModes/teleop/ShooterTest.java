package org.firstinspires.ftc.teamcode.OpModes.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@Configurable
@TeleOp(name = "Shooter Test", group = "test")
public class ShooterTest extends OpMode {
    private Shooter shooter;
    public static double RPM = 0;
    public static double hoodPosition = 0;

    public void init() {
        shooter = new Shooter(hardwareMap);
    }
    public void start() {
        shooter.setManualControl(true);
        shooter.spinUpCommand().schedule();
    }

    public void loop() {

        if (gamepad1.dpad_up) { shooter.spinUpCommand().schedule(); }
        if (gamepad1.dpad_down) { shooter.spinDownCommand().schedule(); }

        if (gamepad1.rightTriggerWasPressed()) {
            shooter.setTargetRPM(RPM);
            shooter.setTargetHoodPosition(hoodPosition);
        }

    }

}
