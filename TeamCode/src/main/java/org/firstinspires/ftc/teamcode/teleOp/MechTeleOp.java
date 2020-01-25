package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;      //robot controlling super class
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;      //annotation for driver station to show this class as a TeleOp class
import com.qualcomm.robotcore.hardware.CRServo;             //continuous rotation servo control class
import com.qualcomm.robotcore.hardware.DcMotor;             //DC Motor controlling class
import com.qualcomm.robotcore.hardware.DcMotorSimple;       //helper class for DC Motor

import com.qualcomm.robotcore.util.Range;                   //class used for calculations

import java.lang.Math;                                      //class used for calculations

@TeleOp(name = "MechTeleOp")
//tells the driver station to show this class as MechTeleOp in TeleOp menu
//@Disabled                     //disable this class
public class MechTeleOp extends OpMode {    //class declaration with OpMode as superclass

    //variables for motors for wheel
    DcMotor motorFL;
    DcMotor motorFR;
    DcMotor motorBL;
    DcMotor motorBR;

    //calibration coefficient (tweak to calibrate the motors (0 < x <= 1))
    final double calibFL = 1.0f;
    final double calibFR = 1.0f;
    final double calibBL = 1.0f;
    final double calibBR = 1.0f;

    //claw motors
    DcMotor clawLeft;
    DcMotor clawRight;

    //lifting motors
    DcMotor leftLift;
    DcMotor rightLift;

    //servo for foundation clip
    CRServo foundationClip;

    //joystick variables
    double leftX, leftY, rightX, rightY;

    //matrix corresponding to motors
//    double[][] motorPowers = {
//            {0.0, 0.0},
//            {0.0, 0.0}
//    };
//
//    //matrix corresponding to
//    double[][] leftXMat = {
//            { 0.0,  0.0},
//            { 0.0,  0.0}
//    };
//
//    double[][] leftYMat = {
//            { 0.0,  0.0},
//            { 0.0,  0.0}
//    };
//
//    double[][] rightXMat = {
//            { 0.0,  0.0},
//            { 0.0,  0.0}
//    };
//
//    double[][] rightYMat = {
//            { 0.0,  0.0},
//            { 0.0,  0.0}
//    };

    //constructor
    public MechTeleOp() {
        super();
    }

    // executed when the "Init" button is pressed
    @Override
    public void init() {
        setupMotors();  // see method declaration
        resetStartTime(); //reset the timer variable
        telemetry.addData("Status", "Initialized"); // send a message to the driver station to say the robot has been initialized
        telemetry.update(); // update the driver station's message feed
    }

    //executed over and over when the "start triangle" is pressed
    @Override
    public void loop() {
        driveRobot(); // see method declaration
        telemetry.update(); // update the driver station's message feed
    }

    //executed when the "stop squar" is pressed
    @Override
    public void stop() {
        moveForward(0.0);
        clawLeft.setPower(0.0);
        clawRight.setPower(0.0);
        leftLift.setPower(0.0);
        rightLift.setPower(0.0);
        foundationClip.setPower(0.0);
    } // stop all motors/servos

    //set drivetrain motors' power to move robot forward. negative parameter moves backwards
    public void moveForward(double power) {
        motorFL.setPower(Range.clip(-calibFL * power, -1, 1));
        motorFR.setPower(Range.clip(-calibFR * power, -1, 1));
        motorBL.setPower(Range.clip(-calibBL * power, -1, 1));
        motorBR.setPower(Range.clip(-calibBR * power, -1, 1));
    }

    //set drivetrain motors' power to rotate the robot. negative parameter rotates right
    public void rotateLeft(double power) {
        motorFL.setPower(calibFL * -power);
        motorFR.setPower(calibFR * power);
        motorBL.setPower(calibBL * -power);
        motorBR.setPower(calibBR * power);
    }

    //set drivetrain motors' power to straif towards the left. negative parameter straifs right
    public void straifLeft(double power) {
        motorFL.setPower(calibFL * -power);
        motorFR.setPower(calibFR * -power);
        motorBL.setPower(calibBR * power);
        motorBR.setPower(calibBR * power);
    }

    // set matrices' data to motor power (used along with matrices to control the drivetrain motors)
//    public void matrixToPowers(double[][] power) {
//        motorFL.setPower(calibFL * power[0][0]);
//        motorFR.setPower(calibFR * power[0][1]);
//        motorBL.setPower(calibBL * power[1][0]);
//        motorBR.setPower(calibBR * power[1][1]);
//    }

    // find the average of the matrices (used along with matrices to control the drivetrain motors)
//    public double[][] avgPowerMatrix(double[][] leftY, double[][] leftX, double[][] rightY, double[][]rightX){
//        double[][] result = {
//                { 0.0,  0.0},
//                { 0.0,  0.0}
//        };
//
//        for (int i = 0; i < 2; i++) {
//            for (int k = 0; k < 2; k++) {
//                result[i][k] = leftY[i][k] + leftX[i][k] + rightY[i][k] + rightX[i][k];
//            }
//        }
//
//        for (int i = 0; i < 2; i++) {
//            for (int k = 0; k < 2; k++) {
//                result[i][k] /= 2;
//            }
//        }
//
//        return result;
//    }

    public void setupMotors() {

        ///////////////////////////////////////////////////////////////////////////////// Drive Train
        //assign motors fetched from hardwaremap to motor variables
        motorFL = hardwareMap.get(DcMotor.class, "frontLeft"); // frontLeft
        motorFR = hardwareMap.get(DcMotor.class, "frontRight"); // frontRight
        motorBL = hardwareMap.get(DcMotor.class, "backLeft"); // backLeft
        motorBR = hardwareMap.get(DcMotor.class, "backRight"); // backRight

        //assign direction for each motor
        motorFL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBR.setDirection(DcMotorSimple.Direction.FORWARD);

        //assign run mode for motors
        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ///////////////////////////////////////////////////////////////////////////////// Drive Train

        ///////////////////////////////////////////////////////////////////////////////// Claw
        //assign claw motors to claw variables
        clawLeft = hardwareMap.get(DcMotor.class, "leftClaw");
        clawRight = hardwareMap.get(DcMotor.class, "rightClaw");

        //assign claw motor directions
        clawLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        clawRight.setDirection(DcMotorSimple.Direction.REVERSE);

        //assign claw motors runmode
        clawLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        clawRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ///////////////////////////////////////////////////////////////////////////////// Claw

        ///////////////////////////////////////////////////////////////////////////////// Elevator
        //assign lift motors from hardwaremap to lift variables
        leftLift = hardwareMap.get(DcMotor.class, "leftLift");
        rightLift = hardwareMap.get(DcMotor.class, "rightLift");

        //set lift directions
        leftLift.setDirection(DcMotorSimple.Direction.FORWARD);
        rightLift.setDirection(DcMotorSimple.Direction.REVERSE);

        //set lift motor runmode
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //make the motors brake when not powered
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ///////////////////////////////////////////////////////////////////////////////// Elevator

        ///////////////////////////////////////////////////////////////////////////////// Foundation
        // assign foundation servo to variable
        foundationClip = hardwareMap.crservo.get("foundationClip");
        ///////////////////////////////////////////////////////////////////////////////// Foundation
    }

    //method where all motor controls are done.
    public void driveRobot() {
        updateJoystickVars();   //fetch joystick values from gamepad
        reportJoysticks();      //report joystick values to drive station

        //drivetrain control with matrices for mechanum wheels
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

        controlLift();  //see method declaration

        //extra function: If y is pressed, spit block and move back
        if (gamepad1.y) {
            clawLeft.setPower(-0.4);    //spit block
            clawRight.setPower(-0.4);   //spit block
            moveForward(-0.6);   //move back
        } else {
//            matrixToPowers(motorPowers);
            controlMovement();  //see method declaration
            controlClaws();     //see method declaration
        }

        //foundation clip control
        if (gamepad1.a) {
            foundationClip.setPower(1.0); // drop clip
        } else if (gamepad1.b) {
            foundationClip.setPower(-1.0); // raise clip
        } else {
            foundationClip.setPower(0.0); // stop
        }
    }

    //method used to control lift motors
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

    //method used to control robot drivetrain
    public void controlMovement() {
        if (Math.abs(rightX) > 0.1 || Math.abs(leftY) > 0.1) {
            if (Math.abs(rightX) > Math.abs(leftY)) {
                rotateLeft(Range.clip(rightX, -1, 1));
                telemetry.addData("Drivetrain", "Rotating");
            } else {
                moveForward(Range.clip(leftY, -0.8, 0.8));
                telemetry.addData("Drivetrain", "Forward/Back");
            }
        } else {
            telemetry.addData("Drivetrain", "Stopped");
            moveForward(0.0);
        }
    }

    //method used to control the claw motors
    public void controlClaws() {
        controlRightClaw(); //see method declaration
        controlLeftClaw(); //see method declaration
    }

    //method used to control the right claw motor
    public void controlRightClaw() {
        if (gamepad1.right_trigger >= 0.1) {
            clawRight.setPower(0.5);
        } else if (gamepad1.right_bumper) {
            clawRight.setPower(-0.3);
        } else {
            clawRight.setPower(0.0);
        }

    }

    //method used to control the left claw motor
    public void controlLeftClaw() {
        if (gamepad1.left_trigger >= 0.1) {
            clawLeft.setPower(0.5);
        } else if (gamepad1.left_bumper) {
            clawLeft.setPower(-0.3);
        } else {
            clawLeft.setPower(0.0);
        }
    }

    //fetch gamepad left joystic's x value
    public float getLX() {
        return gamepad1.left_stick_x;
    }

    //fetch gamepad left joystic's t value
    public float getLY() {
        return gamepad1.left_stick_y;
    }

    //fetch gamepad right joystic's x value
    public float getRX() {
        return gamepad1.right_stick_x;
    }

    //fetch gamepad right joystic's t value
    public float getRY() {
        return gamepad1.right_stick_y;
    }

    //fetch joystick values and assign them to their variables
    public void updateJoystickVars() {
        leftX = getLX();
        leftY = -getLY();
        rightX = getRX();
        rightY = getRY();
    }

    //send the values of the joysticks to the driver station
    public void reportJoysticks() {
        telemetry.addData("gamepad LX", getLX());
        telemetry.addData("gamepda LY", -getLY());
        telemetry.addData("gamepad RX", getRX());
        telemetry.addData("gamepda RY", getRY());
    }

}
