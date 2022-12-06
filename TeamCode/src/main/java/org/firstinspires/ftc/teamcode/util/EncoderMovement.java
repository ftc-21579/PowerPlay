package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Allows movement of the robot using encoders
 */
@Config
public class EncoderMovement {
    // TODO: Setup IMU for turning

    private final DcMotor FrontLeft, BackLeft, FrontRight, BackRight;
    private final Telemetry telemetry;
    private int flPos, blPos, frPos, brPos;

    // Config Vars
    public static double ticksPerInch = 114.6;
    public static double ticksPerDeg = 4; // TODO: Verify this number

    public EncoderMovement(HardwareMap hardware, Telemetry givenTelemetry) {

        // Define telemetry
        telemetry = givenTelemetry;

        // Define Motors
        FrontLeft = hardware.dcMotor.get("motorFrontLeft");
        BackLeft = hardware.dcMotor.get("motorBackLeft");
        FrontRight = hardware.dcMotor.get("motorFrontRight");
        BackRight = hardware.dcMotor.get("motorBackRight");

        // Adjust Motor Directions
        FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        BackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set run mode
        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set motor target
        FrontLeft.setTargetPosition(0);
        BackLeft.setTargetPosition(0);
        FrontRight.setTargetPosition(0);
        BackRight.setTargetPosition(0);

        // Set run mode
        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    //region Movement

    /**
     * Will move the robot forwards based on parameters.
     * This method will run immediately.
     *
     * @param distance  The distance (in inches) to move forwards
     * @param speed     The speed at which the motor should spin (0.0 - 1.0)
     */
    public void moveForward(int distance, double speed) {

        // Fetch motor positions
        getMotorPositions();

        // Calculate targets
        flPos += distance * ticksPerInch;
        blPos += distance * ticksPerInch;
        frPos += distance * ticksPerInch;
        brPos += distance * ticksPerInch;

        // Begin Movement
        beginMovement(speed);

        // Telemetry
        while (motorsBusy()) {
            outputTelemetry("Move Forward");
        }

        // Once completed, stop motors
        endMovement();
    }

    /**
     * Will move the robot backwards based on parameters.
     * This method will run immediately.
     *
     * @param distance  The distance (in inches) to move backwards
     * @param speed     The speed at which the motor should spin (0.0 - 1.0)
     */
    public void moveBackward(int distance, double speed) {

        // Fetch motor positions
        getMotorPositions();

        // Calculate targets
        flPos -= distance * ticksPerInch;
        blPos -= distance * ticksPerInch;
        frPos -= distance * ticksPerInch;
        brPos -= distance * ticksPerInch;

        // Begin Movement
        beginMovement(speed);

        // Telemetry
        while (motorsBusy()) {
            outputTelemetry("Move Backward");
        }

        // Once completed, stop motors
        endMovement();
    }

    /**
     * Will strafe the robot to the right based on parameters.
     * This method will run immediately.
     *
     * @param distance  The distance (in inches) to strafe right
     * @param speed     The speed at which the motor should spin (0.0 - 1.0)
     */
    public void strafeRight(int distance, double speed) {

        // Fetch motor positions
        getMotorPositions();

        // Calculate targets
        flPos += distance * ticksPerInch;
        blPos -= distance * ticksPerInch;
        frPos -= distance * ticksPerInch;
        brPos += distance * ticksPerInch;

        // Begin Movement
        beginMovement(speed);

        // Telemetry
        while (motorsBusy()) {
            outputTelemetry("Strafe Right");
        }

        // Once completed, stop motors
        endMovement();
    }

    /**
     * Will strafe the robot left based on parameters.
     * This method will run immediately.
     *
     * @param distance  The distance (in inches) to strafe left
     * @param speed     The speed at which the motor should spin (0.0 - 1.0)
     */
    public void strafeLeft(int distance, double speed) {

        // Fetch motor positions
        getMotorPositions();

        // Calculate targets
        flPos -= distance * ticksPerInch;
        blPos += distance * ticksPerInch;
        frPos += distance * ticksPerInch;
        brPos -= distance * ticksPerInch;

        // Begin Movement
        beginMovement(speed);

        // Telemetry
        while (motorsBusy()) {
            outputTelemetry("Strafe Left");
        }

        // Once completed, stop motors
        endMovement();
    }

    /**
     * Will turn the robot clockwise based on parameters.
     * This method will run immediately.
     * NOTE: THIS METHOD IS UNTESTED
     *
     * @param angle     The distance (in inches) to move forwards
     * @param speed     The speed at which the motor should spin (0.0 - 1.0)
     */
    public void turnClockwise(int angle, double speed) {

        // Fetch motor positions
        getMotorPositions();

        // Calculate targets
        flPos += angle * ticksPerDeg;
        blPos += angle * ticksPerDeg;
        frPos -= angle * ticksPerDeg;
        brPos -= angle * ticksPerDeg;

        // Begin Movement
        beginMovement(speed);

        // Telemetry
        while (motorsBusy()) {
            outputTelemetry("Turn Clockwise");
        }

        // Once completed, stop motors
        endMovement();
    }

    //endregion




    //region Private Utilities

    private void getMotorPositions() {
        // Sets flPos, etc. to the current positions

        flPos = FrontLeft.getCurrentPosition();
        blPos = BackLeft.getCurrentPosition();
        frPos = FrontRight.getCurrentPosition();
        brPos = BackRight.getCurrentPosition();
    }

    private void beginMovement(double speed) {
        // takes speed as an input (0.0 - 1.0)

        FrontLeft.setTargetPosition(flPos);
        BackLeft.setTargetPosition(blPos);
        FrontRight.setTargetPosition(frPos);
        BackRight.setTargetPosition(brPos);

        FrontLeft.setPower(speed);
        BackLeft.setPower(speed);
        FrontRight.setPower(speed);
        BackRight.setPower(speed);
    }

    private void endMovement() {
        FrontLeft.setPower(0);
        BackLeft.setPower(0);
        FrontRight.setPower(0);
        BackRight.setPower(0);
    }

    private boolean motorsBusy() {
        return(FrontLeft.isBusy() && BackLeft.isBusy() && FrontRight.isBusy() && BackRight.isBusy());
    }

    private void outputTelemetry(String name) {
        telemetry.addLine(name);
        telemetry.addData("Target", "fl: " + flPos + " bl: " +
                blPos + " fr: " + frPos + " br: " + brPos);
        telemetry.addData("Actual",
                "fl: " + FrontLeft.getCurrentPosition() +
                        " bl: " + BackLeft.getCurrentPosition() +
                        " fr: " + FrontRight.getCurrentPosition() +
                        " br: " + BackRight.getCurrentPosition());
        telemetry.update();
    }
    //endregion
}