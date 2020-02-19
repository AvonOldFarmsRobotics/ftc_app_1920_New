package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "TurnLeftToBridge")
public class TurnLeftToBridge extends RobotController {

    @Override
    public void runOpMode() throws InterruptedException {
        initMotors();
        waitForStart();


        stop();
    }
}
