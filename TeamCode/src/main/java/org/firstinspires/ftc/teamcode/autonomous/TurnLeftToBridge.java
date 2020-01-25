package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "TurnLeftToBridge")
public class TurnLeftToBridge extends RobotController {

    @Override
    public void runOpMode() throws InterruptedException {
        initMotors();
        waitForStart();

        moveForwardRaw(0.5);
        sleep(500);
        moveForwardRaw(0.0);
        sleep(1500);
        rotateLeftRaw(0.5);
        sleep(850);
        rotateLeftRaw(0.0);
        sleep(1500);
        moveForwardRaw(0.5);
        sleep(1000);
        moveForwardRaw(0.0);

        stop();
    }
}
