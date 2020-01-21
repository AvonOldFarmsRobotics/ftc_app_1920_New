package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue Loading Zone")
public class BlueLoadingZoneAutonomous extends RobotController {

    public void runOpMode() {
        waitForStart();

        doAction("forward", 4);
        //grab block?
        doAction("rotate", -5);
        doAction("forward", 5);

        stop();
    }

}
