package org.firstinspires.ftc.teamcode.OpModes;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.Alliance;
import org.firstinspires.ftc.teamcode.utils.RobotLogger;

public abstract class CommandOpMode extends OpMode {
    protected Robot robot;
    protected RobotLogger logger;
    private static Alliance alliance = Alliance.BLUE;
    protected Telemetry telemetry;

    protected static Alliance getAlliance() {
        return alliance;
    }
    public static void setAlliance(Alliance alliance) {
        CommandOpMode.alliance = alliance;
    }

    @Override
    public void init() {
        Scheduler.reset();

        robot = new Robot(hardwareMap, alliance);
        logger = new RobotLogger(robot);
        telemetry = new JoinedTelemetry(
                super.telemetry,
                PanelsTelemetry.INSTANCE.getFtcTelemetry()
        );

        logger.setIntakeLogging(true);
        logger.setShooterLogging(true);
        logger.setTurretLogging(true);
    }

    @Override
    public void init_loop() {
        if (gamepad1.dpad_left) {
            alliance = Alliance.BLUE;
        } else if (gamepad1.dpad_right) {
            alliance = Alliance.RED;
        }
        robot.alliance = alliance;

        telemetry.addLine("Select Alliance, press dpad left for blue, dpad right for red.");
        telemetry.addData("alliance: ", robot.alliance);
        telemetry.update();
    }

    @Override
    public void start() {
        robot.start();
    }

    public void commandLoop(Runnable command) {
        robot.clearCache();

        command.run();
        robot.periodic();

        telemetry.addData("Loop time (HZ) : ",robot.getFrequency());

        logger.update();
        telemetry.update();
    }

    @Override
    public void stop() {
        robot.savePose(robot.follower.pose());
    }
}
