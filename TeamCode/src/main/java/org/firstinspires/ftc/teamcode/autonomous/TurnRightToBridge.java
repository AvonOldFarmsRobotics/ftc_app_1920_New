package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Forward Just A Bit")
public class TurnRightToBridge extends RobotController {
    @Override
    public void runOpMode() throws InterruptedException {
        initMotors();
        waitForStart();

        moveForwardRaw(0.5);
        sleep(315);
        moveForwardRaw(0.0);
        sleep(297000);
    }
}
