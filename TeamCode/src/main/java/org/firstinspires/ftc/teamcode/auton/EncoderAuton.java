package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.util.EncoderMovement;

@Config
@Autonomous(name="Parking", group="LinearOpmode")
public class EncoderAuton extends LinearOpMode {

    /*// Declare motors
    DcMotor motorFrontLeft;
    DcMotor motorBackLeft;
    DcMotor motorFrontRight;
    DcMotor motorBackRight;

    // Motor position variables
    private int flPos, blPos, frPos, brPos;

    // Operational Constants
    private final double fast = 0.7;
    private final double medium = 0.4;
    private final double slow = 0.2;
    public static double ticksPerInch = 114.6;
    public static double ticksPerDeg = 4; // TODO: Verify this number

    public static int distanceOne = 27;
    public static int distanceTwo = 26;

    @Override
    public void runOpMode() {
        telemetry.setAutoClear(true);

        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");

        // Adjust motor directions
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set motor run modes
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set motor target
        motorFrontLeft.setTargetPosition(0);
        motorBackLeft.setTargetPosition(0);
        motorFrontRight.setTargetPosition(0);
        motorBackRight.setTargetPosition(0);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Wait for play pressing
        waitForStart();

        moveForward(distanceOne, medium);
        moveRight(distanceTwo, slow);
    }

    private void moveForward(int distance, double speed) {
        // Distance is in inches. Negative amount moves backwards.

        // Fetch motor positions
        flPos = motorFrontLeft.getCurrentPosition();
        blPos = motorBackLeft.getCurrentPosition();
        frPos = motorFrontRight.getCurrentPosition();
        brPos = motorBackRight.getCurrentPosition();

        // Calculate new targets
        flPos += distance * ticksPerInch;
        blPos += distance * ticksPerInch;
        frPos += distance * ticksPerInch;
        brPos += distance * ticksPerInch;

        // Move robot to new position
        motorFrontLeft.setTargetPosition(flPos);
        motorBackLeft.setTargetPosition(blPos);
        motorFrontRight.setTargetPosition(frPos);
        motorBackRight.setTargetPosition(brPos);

        motorFrontLeft.setPower(speed);
        motorBackLeft.setPower(speed);
        motorFrontRight.setPower(speed);
        motorBackRight.setPower(speed);

        // Wait for movement to finish
        while (motorFrontLeft.isBusy() && motorBackLeft.isBusy() &&
                motorFrontRight.isBusy() && motorBackRight.isBusy()) {

            // Output some telemetry
            telemetry.addLine("Move Forward");
            telemetry.addData("Target", "fl: " + Integer.toString(flPos) + " bl: " +
                    Integer.toString(blPos) + " fr: " + Integer.toString(frPos) + " br: " +
                    Integer.toString(brPos));
            telemetry.addData("Actual",
                    "fl: " + Integer.toString(motorFrontLeft.getCurrentPosition()) +
                            " bl: " + Integer.toString(motorBackLeft.getCurrentPosition()) +
                            " fr: " + Integer.toString(motorFrontRight.getCurrentPosition()) +
                            " br: " + Integer.toString(motorBackRight.getCurrentPosition()));
            telemetry.update();
        }

        // After done, stop all motors
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
    }

    private void moveRight(int distance, double speed) {
        // distance is in inches. A negative distance moves left

        // Fetch motor positions
        flPos = motorFrontLeft.getCurrentPosition();
        blPos = motorBackLeft.getCurrentPosition();
        frPos = motorFrontRight.getCurrentPosition();
        brPos = motorBackRight.getCurrentPosition();

        // Calculate new targets
        flPos += distance * ticksPerInch;
        blPos -= distance * ticksPerInch;
        frPos -= distance * ticksPerInch;
        brPos += distance * ticksPerInch;

        // Move robot to new position
        motorFrontLeft.setTargetPosition(flPos);
        motorBackLeft.setTargetPosition(blPos);
        motorFrontRight.setTargetPosition(frPos);
        motorBackRight.setTargetPosition(brPos);

        motorFrontLeft.setPower(speed);
        motorBackLeft.setPower(speed);
        motorFrontRight.setPower(speed);
        motorBackRight.setPower(speed);

        // Wait for movement to finish
        while (motorFrontLeft.isBusy() && motorBackLeft.isBusy() &&
                motorFrontRight.isBusy() && motorBackRight.isBusy()) {

            // Output some telemetry
            telemetry.addLine("Strafe Right");
            telemetry.addData("Target", "fl: " + Integer.toString(flPos) + " bl: " +
                    Integer.toString(blPos) + " fr: " + Integer.toString(frPos) + " br: " +
                    Integer.toString(brPos));
            telemetry.addData("Actual",
                    "fl: " + Integer.toString(motorFrontLeft.getCurrentPosition()) +
                            " bl: " + Integer.toString(motorBackLeft.getCurrentPosition()) +
                            " fr: " + Integer.toString(motorFrontRight.getCurrentPosition()) +
                            " br: " + Integer.toString(motorBackRight.getCurrentPosition()));
            telemetry.update();
        }

        // After done, stop all motors
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
    }

    private void turnClockwise(int angle, double speed) {
        // angle is in degrees. Negative angle will turn counter clockwise

        // Fetch motor positions
        flPos = motorFrontLeft.getCurrentPosition();
        blPos = motorBackLeft.getCurrentPosition();
        frPos = motorFrontRight.getCurrentPosition();
        brPos = motorBackRight.getCurrentPosition();

        // Calculate new targets
        flPos += angle * ticksPerDeg;
        blPos += angle * ticksPerDeg;
        frPos -= angle * ticksPerDeg;
        brPos -= angle * ticksPerDeg;

        // Move robot to new position
        motorFrontLeft.setTargetPosition(flPos);
        motorBackLeft.setTargetPosition(blPos);
        motorFrontRight.setTargetPosition(frPos);
        motorBackRight.setTargetPosition(brPos);

        motorFrontLeft.setPower(speed);
        motorBackLeft.setPower(speed);
        motorFrontRight.setPower(speed);
        motorBackRight.setPower(speed);

        // Wait for movement to finish
        while (motorFrontLeft.isBusy() && motorBackLeft.isBusy() &&
                motorFrontRight.isBusy() && motorBackRight.isBusy()) {

            // Output some telemetry
            telemetry.addLine("Turn Clockwise");
            telemetry.addData("Target", "fl: " + Integer.toString(flPos) + " bl: " +
                    Integer.toString(blPos) + " fr: " + Integer.toString(frPos) + " br: " +
                    Integer.toString(brPos));
            telemetry.addData("Actual",
                    "fl: " + Integer.toString(motorFrontLeft.getCurrentPosition()) +
                            " bl: " + Integer.toString(motorBackLeft.getCurrentPosition()) +
                            " fr: " + Integer.toString(motorFrontRight.getCurrentPosition()) +
                            " br: " + Integer.toString(motorBackRight.getCurrentPosition()));
            telemetry.update();
        }

        // After done, stop all motors
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
    }*/

    private EncoderMovement movement;

    @Override
    public void runOpMode() {
        telemetry.setAutoClear(true);

        movement = new EncoderMovement(hardwareMap, telemetry);

        waitForStart();

        movement.moveForward(24, 0.4);
    }
}
