package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Distance Tester")
public class DistanceTester extends RobotController {

    public void runOpMode() {
        initMotors();
        waitForStart();

        doAction("rotate", -1.5);
        doAction("forward", -3);

        stop();
    }
}
