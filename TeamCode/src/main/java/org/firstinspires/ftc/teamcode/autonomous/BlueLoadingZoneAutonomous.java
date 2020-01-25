package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "Blue Loading Zone")
@Disabled
public class BlueLoadingZoneAutonomous extends RobotController {

    public void runOpMode() {
        initMotors();
        waitForStart();

        doAction("forward", 4);
        //grab block?
        doAction("rotate", -5);
        doAction("forward", 5);

        stop();
    }

}
