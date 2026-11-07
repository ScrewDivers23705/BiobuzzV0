package org.firstinspires.ftc.teamcode.OpModes.teleop.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import gay.zharel.fateweaver.telemetry.Fate;

@TeleOp(name = "FateWeaver Test")
public class FateWeaverTest extends OpMode {
    @Override
    public void init() {
        telemetry.addData("FateWeaver", "Ready");
    }

    @Override
    public void loop() {
        Fate.log("test", "Hello from FateWeaver!");
        telemetry.addData("Status", "Logging data...");
        telemetry.update();
    }
}
