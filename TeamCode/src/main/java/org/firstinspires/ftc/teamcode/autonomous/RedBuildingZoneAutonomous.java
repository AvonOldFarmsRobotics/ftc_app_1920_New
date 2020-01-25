package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "Red Building Zone")
@Disabled
public class RedBuildingZoneAutonomous extends RobotController {

    // {"forward", "backward", "left", "right", "rotate"}

    public void runOpMode() {
        initMotors();
        waitForStart();

        doAction("right", 5);
        doAction("forward", 5);
        //clip foundation
        doAction("rotate", 5);
        doAction("backward", 2);
        doAction("rotate", 5);
        doAction("forward", 2);
        //release clip
        doAction("backward", 7);

        stop();
    }
}





