package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public abstract class RobotController extends LinearOpMode {

    DcMotor motorFL;
    DcMotor motorFR;
    DcMotor motorBL;
    DcMotor motorBR;

    final double calibFL = 1.00f;
    final double calibFR = 1.00f;
    final double calibBL = 1.00f;
    final double calibBR = 1.00f;

    private ElapsedTime runtime = new ElapsedTime();

    final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    final double DRIVE_SPEED = 0.6;
    final double TURN_SPEED = 0.4;

    private final String[] actions = {"forward", "backward", "left", "right", "rotateRight", "rotateLeft"};

    public void initMotors() {
        motorFL = hardwareMap.get(DcMotor.class, "frontLeft"); // frontLeft
        motorFR = hardwareMap.get(DcMotor.class, "frontRight"); // frontRight
        motorBL = hardwareMap.get(DcMotor.class, "backLeft"); // backLeft
        motorBR = hardwareMap.get(DcMotor.class, "backRight"); // backRight

        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBR.setDirection(DcMotorSimple.Direction.FORWARD);

        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void moveForwardRaw(double power) {
        motorFL.setPower(Range.clip(calibFL * power, -1, 1));
        motorFR.setPower(Range.clip(calibFR * power, -1, 1));
        motorBL.setPower(Range.clip(calibBL * power, -1, 1));
        motorBR.setPower(Range.clip(calibBR * power, -1, 1));
    }

    public void rotateLeftRaw(double power) {
        motorFL.setPower(calibFL * -power);
        motorFR.setPower(calibFR * power);
        motorBL.setPower(calibBL * -power);
        motorBR.setPower(calibBR * power);
    }

    public void strafeLeftRaw(double power) {
        motorFL.setPower(calibFL * -power);
        motorFR.setPower(calibFR * -power);
        motorBL.setPower(calibBR * power);
        motorBR.setPower(calibBR * power);
    }

    public void doAction(String action, double distance) {
        int actionInt = -1;
        for (int i = 0; i < 5; i++) {
            if (action.equals(actions[i]))
                actionInt = i;
        }

        switch (actionInt) {
            case 0:
                moveForward(distance, distance * 2);
                break;
            case 1:
                moveForward(-distance, distance * 2);
                break;
            case 2:
                strafeLeft(distance, distance * 2);
                break;
            case 3:
                strafeLeft(-distance, distance * 2);
                break;
            case 4:
                rotateLeft(distance, distance * 2);
                break;
            case 5:
                rotateLeft(-distance, distance * 2);
        }

        sleep(1000);

    }

    public void moveForward(double distance, double timeout) {
        encoderDrive(DRIVE_SPEED, distance, distance, timeout, false);
    }

    public void rotateLeft(double distance, double timeout) {
        encoderDrive(TURN_SPEED, -distance, distance, timeout, false);
    }

    public void strafeLeft(double distance, double timeout) {
        encoderDrive(DRIVE_SPEED, -distance, -distance, timeout, true);
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS,
                             boolean strafe) {
        int newFL;
        int newFR;
        int newBL;
        int newBR;

        int strafeCoef = strafe ? -1 : 1;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFL = motorFL.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH * strafeCoef);
            newFR = motorFR.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH * strafeCoef);
            newBL = motorBL.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newBR = motorBR.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);

            motorFL.setTargetPosition(newFL);
            motorFR.setTargetPosition(newFR);
            motorBL.setTargetPosition(newBL);
            motorBR.setTargetPosition(newBR);

            // Turn On RUN_TO_POSITION
            motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            motorFL.setPower(Math.abs(speed) * calibFL);
            motorFR.setPower(Math.abs(speed) * calibFR);
            motorBL.setPower(Math.abs(speed) * calibBL);
            motorBR.setPower(Math.abs(speed) * calibBR);

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (motorFL.isBusy() || motorFR.isBusy() || motorBL.isBusy() || motorBR.isBusy())) {
            }

            // Stop all motion;
            motorFL.setPower(0.0);
            motorFR.setPower(0.0);
            motorBL.setPower(0.0);
            motorBR.setPower(0.0);

            // Turn off RUN_TO_POSITION
            motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}
