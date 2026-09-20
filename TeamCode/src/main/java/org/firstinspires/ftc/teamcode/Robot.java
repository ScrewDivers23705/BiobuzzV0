package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.pedro.Constants.create;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.utils.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.utils.Alliance;

import java.util.List;


public class Robot {
    private List<LynxModule> lynxHubs; // set to manual caching mode to improve looptimes
    private Follower follower; // make follower public so it can be accessed in opmodes
    public Intake intake; // make intake public so it can be accessed in opmodes
    public Shooter shooter; // make shooter public so it can be accessed in opmodes
    public static Pose endPose = Pose.zero(); // make endPose public so it can be accessed in opmodes and drive initialization
    public Alliance alliance; // make alliance public so it can be accessed in opmodes
    private final Timer loop = new Timer();  // make timer object for looptime calculation
    public double loops = 0, lastLoop = 0, loopTime = 0; // make loopTime values for calculation

    public Robot(HardwareMap hardwareMap, Alliance alliance) {
        this.alliance = alliance; // set alliance to the alliance passed in from the opmode

        lynxHubs = hardwareMap.getAll(LynxModule.class); // get all lynx hubs in the robot and set them to manual caching mode to improve looptimes
        for (LynxModule hub : lynxHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        follower = create(hardwareMap); // create the follower object using the create method from the Constants class
        intake = new Intake(hardwareMap); // create the intake object using the hardwareMap passed in from the opmode
        shooter = new Shooter(hardwareMap); // create the shooter object using the hardwareMap passed in from the opmode

        loop.reset(); // reset the timer object for looptime calculation
    }

    public void update(){
        for (LynxModule hub : lynxHubs) hub.clearBulkCache(); // clear the bulk cache for all lynx hubs to improve looptimes

        loops++; // increment loops for looptime calculation

        if (loops >= 15) { // only calculate looptime after 15 loops to avoid initial startup lag
            double curTime = loop.milliseconds(); // get the current time in milliseconds
            loopTime = (curTime - lastLoop) / loops; // calculate the average looptime in milliseconds
            lastLoop = curTime; // set lastLoop to the current time for the next calculation
            loops = 0; // reset loops for the next calculation
        }
    }

    public double getLoopTime(){
        return 1000/ loopTime; // return the average looptime in hertz
    }

    public void savePose(Pose pose){
        endPose = pose; // save the current pose to the static endPose so it can be used for tele
    }
}
