package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name="Parking", group="LinearOpmode")
public class EncoderAuton extends LinearOpMode {

    // Declare motors
    DcMotor motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
    DcMotor motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
    DcMotor motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
    DcMotor motorBackRight = hardwareMap.dcMotor.get("motorBackRight");

    // Motor position variables
    private int flPos, blPos, frPos, brPos;

    // Operational Constants
    private final double fast = 0.7;
    private final double medium = 0.4;
    private final double slow = 0.2;
    private final double ticksPerInch = 114.6; // TODO: Verify this number
    private final double ticksPerDeg = 4; // TODO: Verify this number

    @Override
    public void runOpMode() {
        telemetry.setAutoClear(true);

        // Adjust motor directions
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set motor run modes
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Wait for play pressing
        waitForStart();

        // Instructions here
        // Distance in inches, angles in deg
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
            telemetry.addData("Target", "%7 :%7d", flPos, blPos, frPos, brPos);
            telemetry.addData("Actual", "%7 :%7d",
                    motorFrontLeft.getCurrentPosition(), motorBackLeft.getCurrentPosition(),
                    motorFrontRight.getCurrentPosition(), motorBackRight.getCurrentPosition());
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
        flPos -= distance * ticksPerInch;
        blPos += distance * ticksPerInch;
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
            telemetry.addData("Target", "%7 :%7d", flPos, blPos, frPos, brPos);
            telemetry.addData("Actual", "%7 :%7d",
                    motorFrontLeft.getCurrentPosition(), motorBackLeft.getCurrentPosition(),
                    motorFrontRight.getCurrentPosition(), motorBackRight.getCurrentPosition());
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
            telemetry.addData("Target", "%7 :%7d", flPos, blPos, frPos, brPos);
            telemetry.addData("Actual", "%7 :%7d",
                    motorFrontLeft.getCurrentPosition(), motorBackLeft.getCurrentPosition(),
                    motorFrontRight.getCurrentPosition(), motorBackRight.getCurrentPosition());
            telemetry.update();
        }

        // After done, stop all motors
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
    }
}
