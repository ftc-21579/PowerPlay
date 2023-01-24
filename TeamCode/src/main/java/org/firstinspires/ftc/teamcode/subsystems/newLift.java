package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class newLift {

    private final Telemetry telemetry;
    private final DcMotor liftMotor;
    private final Servo claw;

    private final double ticksPerInch = 64.68056888;
    private int liftPos;

    public newLift(HardwareMap hardware, Telemetry givenTelemetry) {

        // Define Telemetry
        telemetry = givenTelemetry;

        // Define Lift Motor + Reverse it
        liftMotor = hardware.dcMotor.get("liftMotor");
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Define claw
        claw = hardware.servo.get("manipulator");
    }

    public void up(int height, double speed) {

        // Fetch lift position
        getLiftPosition();

        // Calculate target
        liftPos += height * ticksPerInch;

        beginLift(speed);

        while(liftBusy()) {
            telemetry.addData("Lift", "Raising");
        }

        stopLift();
    }

    private void getLiftPosition() {
        liftPos = liftMotor.getCurrentPosition();
    }

    private void beginLift(double speed) {
        liftMotor.setTargetPosition(liftPos);
        liftMotor.setPower(speed);
    }

    private boolean liftBusy() {
        return(liftMotor.isBusy());
    }

    private void stopLift() {
        liftMotor.setPower(0);
    }
}
