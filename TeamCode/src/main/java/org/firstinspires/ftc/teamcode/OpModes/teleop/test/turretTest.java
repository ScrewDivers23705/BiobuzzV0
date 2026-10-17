package org.firstinspires.ftc.teamcode.OpModes.teleop.test;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.TempUnit;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

import java.util.List;

@Configurable
@TeleOp(name = "Turret Test", group = "test")
public class turretTest extends OpMode {
    private Turret turret;
    private List<LynxModule> hubs;
    public static double targetAngleDegrees = 0;
    public static double stepDegrees = 1;


    @Override
    public void init() {
        turret = new Turret(hardwareMap);
        Scheduler.reset();
        hubs = hardwareMap.getAll(LynxModule.class);
    }

    @Override
    public void start() {
        turret.periodic().schedule();
    }

    @Override
    public void loop() {

        if (gamepad1.crossWasPressed()) { targetAngleDegrees = 0; }
        if (gamepad1.circleWasPressed()) { targetAngleDegrees = 90; }
        if (gamepad1.squareWasPressed()) { targetAngleDegrees = -90; }
        if (gamepad1.triangleWasPressed()) { targetAngleDegrees = 180; }
        if (gamepad1.leftBumperWasPressed()) { targetAngleDegrees = -180; }
        if (gamepad1.dpadUpWasPressed()) { targetAngleDegrees += stepDegrees; }
        if (gamepad1.dpadDownWasPressed()) { targetAngleDegrees -= stepDegrees; }
        if (gamepad1.dpadLeftWasPressed()) { targetAngleDegrees = Turret.minAngleDegrees; }
        if (gamepad1.dpadRightWasPressed()) { targetAngleDegrees = Turret.maxAngleDegrees; }

        turret.setTargetAngleDegrees(targetAngleDegrees);
        Scheduler.execute();

        telemetry.addData("Target Angle (deg)", turret.getTargetAngleDegrees());
        telemetry.addData("Current Angle (deg)", turret.getCurrentAngleDegrees());
        telemetry.addData("Estimated Angle (deg)", turret.getEstimatedAngle());
        telemetry.addData("Is Ready", turret.isReady());
        for (LynxModule hub : hubs) {
            telemetry.addData("hub: " + hub.getModuleAddress() + " temp: ", hub.getTemperature(TempUnit.CELSIUS));
            telemetry.addData("hub: " + hub.getModuleAddress() + " curr: ", hub.getCurrent(CurrentUnit.AMPS));
        }
        telemetry.update();

    }
}
