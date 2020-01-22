package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red Loading Zone")
public class RedLoadingZoneAutonomous extends RobotController {

    public void runOpMode() {
        initMotors();
        waitForStart();

        doAction("forward", 4);
        //grab block?
        doAction("rotate", 5);
        doAction("forward", 5);

        stop();
    }

}
