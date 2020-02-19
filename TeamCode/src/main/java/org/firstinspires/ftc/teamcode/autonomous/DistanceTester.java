package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Distance Tester")
public class DistanceTester extends RobotController {

    private final String[] actions = {"forward", "backward", "left", "right", "rotateRight", "rotateLeft"};

    public void runOpMode() {
        initMotors();
        waitForStart();

        int action = 0;
        float distance = 0;

        while (opModeIsActive()) {

            telemetry.addData("Mode", "selecting action");
            telemetry.addData("action", action);
            telemetry.addData("distance", distance);
            telemetry.update();

            if (gamepad1.a) {
                while (gamepad1.a) ;
                if (action < 6)
                    action++;
            }
            if (gamepad1.y) {
                while (gamepad1.y) ;
                if (action > 0)
                    action--;
            }
            if (gamepad1.x) {
                while (gamepad1.x) ;
                if (distance < 10) ;
                distance += 0.1;
            }
            if (gamepad1.b) {
                while (gamepad1.b) ;
                if (distance > 0.0)
                    distance -= 0.1;
            }
            if (gamepad1.right_bumper) {
                while (gamepad1.right_bumper) ;
                telemetry.update();
                telemetry.addData("Mode", "doing action");
                telemetry.update();
                doAction(actions[action], distance);
            }
        }

        stop();
    }
}
