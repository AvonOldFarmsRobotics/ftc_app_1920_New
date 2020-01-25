package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.qualcomm.robotcore.util.Range;

import java.lang.Math;

@TeleOp(name = "MechTeleOp")
//@Disabled
public class MechTeleOp extends OpMode {

    //declare variables for motors for wheel
    DcMotor motorFL;
    DcMotor motorFR;
    DcMotor motorBL;
    DcMotor motorBR;

    //calibration coefficient (tweak to calibrate the motors (0 <= x <= 1))
    final double calibFL = 1.0f;
    final double calibFR = 1.0f;
    final double calibBL = 1.0f;
    final double calibBR = 1.0f;

    //claw motors
    DcMotor clawLeft;
    DcMotor clawRight;

    //elevator
    DcMotor leftLift;
    DcMotor rightLift;

    //servo for foundation clip
    CRServo foundationClip;

    double leftX, leftY, rightX, rightY;

    //matrix corresponding to motors
    double[][] motorPowers = {
            {0.0, 0.0},
            {0.0, 0.0}
    };

    //matrix corresponding to
    double[][] leftXMat = {
            { 0.0,  0.0},
            { 0.0,  0.0}
    };

    double[][] leftYMat = {
            { 0.0,  0.0},
            { 0.0,  0.0}
    };

    double[][] rightXMat = {
            { 0.0,  0.0},
            { 0.0,  0.0}
    };

    double[][] rightYMat = {
            { 0.0,  0.0},
            { 0.0,  0.0}
    };

    public MechTeleOp() {
        super();
    }

    // When the
    @Override
    public void init() {
        setupMotors();
        resetStartTime();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        driveRobot();
        telemetry.update();
    }

    @Override
    public void stop() {
        moveForward(0.0);
    }

    public void moveForward(double power) {
        motorFL.setPower(Range.clip(-calibFL * power, -1, 1));
        motorFR.setPower(Range.clip(-calibFR * power, -1, 1));
        motorBL.setPower(Range.clip(-calibBL * power, -1, 1));
        motorBR.setPower(Range.clip(-calibBR * power, -1, 1));
    }

    public void rotateLeft(double power) {
        motorFL.setPower(calibFL * -power);
        motorFR.setPower(calibFR * power);
        motorBL.setPower(calibBL * -power);
        motorBR.setPower(calibBR * power);
    }

    public void straifLeft(double power) {
        motorFL.setPower(calibFL * -power);
        motorFR.setPower(calibFR * -power);
        motorBL.setPower(calibBR * power);
        motorBR.setPower(calibBR * power);
    }

    public void matrixToPowers(double[][] power) {
        motorFL.setPower(calibFL * power[0][0]);
        motorFR.setPower(calibFR * power[0][1]);
        motorBL.setPower(calibBL * power[1][0]);
        motorBR.setPower(calibBR * power[1][1]);
    }

    public double[][] avgPowerMatrix(double[][] leftY, double[][] leftX, double[][] rightY, double[][]rightX){
        double[][] result = {
                { 0.0,  0.0},
                { 0.0,  0.0}
        };

        for (int i = 0; i < 2; i++) {
            for (int k = 0; k < 2; k++) {
                result[i][k] = leftY[i][k] + leftX[i][k] + rightY[i][k] + rightX[i][k];
            }
        }

        for (int i = 0; i < 2; i++) {
            for (int k = 0; k < 2; k++) {
                result[i][k] /= 2;
            }
        }

        return result;
    }

    public void setupMotors() {

        ///////////////////////////////////////////////////////////////////////////////// Drive Train
        motorFL = hardwareMap.get(DcMotor.class, "frontLeft"); // frontLeft
        motorFR = hardwareMap.get(DcMotor.class, "frontRight"); // frontRight
        motorBL = hardwareMap.get(DcMotor.class, "backLeft"); // backLeft
        motorBR = hardwareMap.get(DcMotor.class, "backRight"); // backRight

        motorFL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBR.setDirection(DcMotorSimple.Direction.FORWARD);

        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ///////////////////////////////////////////////////////////////////////////////// Drive Train

        ///////////////////////////////////////////////////////////////////////////////// Claw
        clawLeft = hardwareMap.get(DcMotor.class, "leftClaw");
        clawRight = hardwareMap.get(DcMotor.class, "rightClaw");

        clawLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        clawRight.setDirection(DcMotorSimple.Direction.REVERSE);

        clawLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        clawRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ///////////////////////////////////////////////////////////////////////////////// Claw

        ///////////////////////////////////////////////////////////////////////////////// Elevator
        leftLift = hardwareMap.get(DcMotor.class, "leftLift");
        rightLift = hardwareMap.get(DcMotor.class, "rightLift");

        leftLift.setDirection(DcMotorSimple.Direction.FORWARD);
        rightLift.setDirection(DcMotorSimple.Direction.REVERSE);

        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ///////////////////////////////////////////////////////////////////////////////// Elevator

        foundationClip = hardwareMap.crservo.get("foundationClip");
    }

    public void driveRobot() {

        updateJoystickVars();
        reportJoysticks();
//
//        if(Math.max(Math.max(Math.abs(leftX), Math.abs(leftY)), Math.max(Math.abs(rightX), Math.abs(rightY))) > 0.1){ // Makes sure that atleast one of the sticks is being pressed
//            /* If we stick with a plain averaging system, both joysticks have to be
//             * pressed forward or backwards to go max speed. This will also effect
//             * the speeds of straifing and turning. A counterbalance can be added
//             * to offset that effect.*/
//
//            leftXMat = new double[][]{  //straif
//                    {leftX * 2, -leftX * 2}, //Here the negative will probably have to on the bottom motors
//                    {-leftX * 1, leftX * 1}
//            };
//
//            leftYMat = new double[][] { //forward/backward
//                    { leftY,  leftY},
//                    { leftY,  leftY}
//            };
//
//            rightXMat = new double[][] { //rotate
//                    {rightX, -rightX},
//                    {rightX, -rightX}
//            };
//
//            rightYMat = new double[][] { //forward/backward
//                    { rightY,  rightY},
//                    { rightY,  rightY}
//            };
//
//            motorPowers = avgPowerMatrix(leftYMat, leftXMat, rightYMat, rightXMat);
//
//        }else{
//            motorPowers = new double[][] {
//                { 0.0,  0.0},
//                { 0.0,  0.0}
//            };
//        }

        controlLift();

        //extra function: If y is pressed, spit block and move back
        if (gamepad1.y) {
            clawLeft.setPower(-0.3);
            clawRight.setPower(-0.3);
            moveForward(-0.6);
        } else {
//            matrixToPowers(motorPowers);
            controlMovement();
            controlClaws();
        }

        //foundation clip control
        if (gamepad1.a) {
            foundationClip.setPower(1.0);
        } else if (gamepad1.b) {
            foundationClip.setPower(-1.0);
        } else {
            foundationClip.setPower(0.0);
        }
    }

    public void controlLift() {
        if (rightY > 0.1) {
            leftLift.setPower(0.5);
            rightLift.setPower(0.5);
        } else if (rightY < -0.1) {
            leftLift.setPower(-0.5);
            rightLift.setPower(-0.5);
        } else {
            leftLift.setPower(0.0);
            rightLift.setPower(0.0);
        }
    }

    public void controlMovement() {
        if (Math.abs(leftX) > 0.1 || Math.abs(leftY) > 0.1) {
            if (Math.abs(leftX) > Math.abs(leftY)) {
                rotateLeft(leftX);
                telemetry.addData("Drivetrain", "Rotating");
            } else {
                moveForward(leftY);
                telemetry.addData("Drivetrain", "Forward/Back");
            }
        } else {
            telemetry.addData("Drivetrain", "Stopped");
            moveForward(0.0);
        }
    }

    public void controlClaws() {
        controlRightClaw();
        controlLeftClaw();
    }

    public void controlRightClaw() {
        if (gamepad1.right_trigger >= 0.1) {
            clawRight.setPower(0.5);
        } else if (gamepad1.right_bumper) {
            clawRight.setPower(-0.3);
        } else {
            clawRight.setPower(0.0);
        }

    }

    public void controlLeftClaw() {
        if (gamepad1.left_trigger >= 0.1) {
            clawLeft.setPower(0.5);
        } else if (gamepad1.left_bumper) {
            clawLeft.setPower(-0.3);
        } else {
            clawLeft.setPower(0.0);
        }
    }

    public float getLX() {
        return gamepad1.left_stick_x;
    }

    public float getLY() {
        return gamepad1.left_stick_y;
    }

    public float getRX() {
        return gamepad1.right_stick_x;
    }
    
    public float getRY() {
        return gamepad1.right_stick_y;
    }

    public void updateJoystickVars() {
        leftX = getLX();
        leftY = -getLY();
        rightX = getRX();
        rightY = getRY();
    }

    public void reportJoysticks() {
        telemetry.addData("gamepad LX", getLX());
        telemetry.addData("gamepda LY", -getLY());
    }

}
