package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.pedro.Constants.create;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.utils.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.utils.Alliance;
import org.psilynx.psikit.core.Logger;

import java.util.List;
import java.util.Map;


public class Robot {
    private List<LynxModule> lynxHubs; // set to manual caching mode to improve looptimes
    private final VoltageSensor voltageSensor;
    public Follower follower; // make follower public so it can be accessed in opmodes
    public Intake intake; // make intake public so it can be accessed in opmodes
    public Shooter shooter; // make shooter public so it can be accessed in opmodes
    public Turret turret; // make turret public so it can be accessed in opmodes
    public static Pose endPose = Pose.zero(); // make endPose public so it can be accessed in opmodes and drive initialization
    public Alliance alliance; // make alliance public so it can be accessed in opmodes
    private final Timer loop = new Timer();  // make timer object for looptime calculation
    public double loops = 0, lastLoop = 0, loopTime = 1; // make loopTime values for calculation
    public double voltage = 12; // make voltage public so it can be accessed in opmodes

    /** Main Constructor for the Robot class. Initializes all subsystems and sets up the robot's hardware.
     * @param hardwareMap The hardware map to use for the robot's hardware.
     * @param alliance The alliance to use for the robot (can be changed later via robot.setAlliance()).
     */
    public Robot(HardwareMap hardwareMap, Alliance alliance) {
        Scheduler.reset(); // reset the scheduler to clear any previous commands
        this.alliance = alliance; // set alliance to the alliance passed in from the opmode

        lynxHubs = hardwareMap.getAll(LynxModule.class); // get all lynx hubs in the robot and set them to manual caching mode to improve looptimes
        for (LynxModule hub : lynxHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        follower = create(hardwareMap).withLogger(log -> {
            logMap("Follower/State", log.followState());
            logMap("Follower/Localizer", log.localizer());
            logMap("Follower/Drivetrain", log.drivetrain());
            logMap("Follower/Algorithm", log.algorithm());
        }); // create the follower object using the create method from the Constants class

        intake = new Intake(hardwareMap); // create the intake object using the hardwareMap passed in from the opmode
        shooter = new Shooter(hardwareMap); // create the shooter object using the hardwareMap passed in from the opmode
        turret = new Turret(hardwareMap); // create the turret object using the hardwareMap passed in from the opmode

        this.voltageSensor = hardwareMap.voltageSensor.iterator().next(); // get the voltage sensor from the hardware map
        this.voltage = voltageSensor.getVoltage(); // get the current voltage of the robot
        shooter.setCurrentVoltage(voltage);

        loop.reset(); // reset the timer object for looptime calculation
    }

    /** Periodic method for the robot class. This method is called every loop and updatse the all the subsystems.
     * it also clears the  cache for all lynx hubs for better looptimes. (:
     */
    public void periodic(){
        loops++; // increment loops for looptime calculation

        follower.update(); // update the follower object to update the robot's pose
        Scheduler.execute(); // execute the scheduler to run any    commands that are scheduled

        if (lastLoop == 0) {
            lastLoop = loop.milliseconds();
        }


        if (loops >= 25) { // only calculate looptime after 25 loops to avoid initial startup lag
            double curTime = loop.milliseconds(); // get the current time in milliseconds
            loopTime = (curTime - lastLoop) / loops; // calculate the average looptime in milliseconds
            lastLoop = curTime; // set lastLoop to the current time for the next calculation
            loops = 0; // reset loops for the next calculation

            voltage = voltageSensor.getVoltage(); // get the current voltage of the robot (call only every 15 loops to avoid lag)
            shooter.setCurrentVoltage(voltage);
        }
    }

    /** Start method for the robot. should be called once in the start() of the opmode to schedule all the periodic subsystem comamnds. */
    public void start(){
        intake.setAlliance(alliance);
        Scheduler.schedule(
            shooter.periodic(),// update the shooter object to update the shooter's state
            turret.periodic(), // update the turret object to update the turret's state
            intake.periodic() // update the intake object to update the intake's state
        );
    }

    /** method to clear the cache for all lynx hubs. */
    public void clearCache() {
        for (LynxModule hub : lynxHubs) {
            hub.clearBulkCache();
        }
    }
    /** method to get hz from the average loop time. */
    public double getFrequency(){
        return 1000/ loopTime; // return the average looptime in hertz
    }
    /** method to save the current pose of the robot to the static endPose variable. */
    public void savePose(Pose pose){
        endPose = pose; // save the current pose to the static endPose so it can be used for tele
    }

    /** utility method to log a map of values to the logger for pedro follower logging */
    private void logMap(String prefix, Map<String, Object> map) {
        for (Map.Entry<String, Object> entry : map.entrySet()) {
            if (entry.getValue() instanceof Double) {
                Logger.recordOutput(prefix + "/" + entry.getKey(), (Double) entry.getValue());
            }
            else if (entry.getValue() instanceof Integer) {
                Logger.recordOutput(prefix + "/" + entry.getKey(), (Integer) entry.getValue());
            }
            else if (entry.getValue() instanceof Boolean) {
                Logger.recordOutput(prefix + "/" + entry.getKey(), (Boolean) entry.getValue());
            }
            else if (entry.getValue() != null){
                Logger.recordOutput(prefix + "/" + entry.getKey(), entry.getValue().toString());
            }
        }
    }
}
