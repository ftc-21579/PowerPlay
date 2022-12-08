package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Drivetrain {

    //region Variable Init
    private final Telemetry telemetry;
    private final DcMotor FrontLeft, BackLeft, FrontRight, BackRight;

    private boolean preciseMode = false;

    public Drivetrain(HardwareMap hardware, Telemetry givenTelemetry) {

        // Define Telemetry
        telemetry = givenTelemetry;

        // Define Motors
        FrontLeft = hardware.dcMotor.get("motorFrontLeft");
        BackLeft = hardware.dcMotor.get("motorBackLeft");
        FrontRight = hardware.dcMotor.get("motorFrontRight");
        BackRight = hardware.dcMotor.get("motorBackRight");

        // Adjust Motor Directions
        FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        BackRight.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    //endregion

    //region Public Methods

    public void setDrivePower(double y, double x, double rx) {
        x = x * 1.1; // Counteracts imperfect strafing

        double max = 1.0;
        if (preciseMode) {
            max = 0.5;
        }

        // Denominator is the largest motor power (absolute value) or max
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-max, max]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), max);

        double flPower = (y + x + rx) / denominator;
        double blPower = (y - x + rx) / denominator;
        double frPower = (y - x - rx) / denominator;
        double brPower = (y + x - rx) / denominator;

        FrontLeft.setPower(flPower);
        BackLeft.setPower(blPower);
        FrontRight.setPower(frPower);
        BackRight.setPower(brPower);

        telemetry.addLine("Motor Status:");
        telemetry.addData("Precise Mode: ", preciseMode);
        telemetry.addData("Motor Powers:", "fl: " + flPower + " bl: " + blPower + " fr: " + frPower + " br: " + brPower);
    }

    public void precisionMode() {
        if (preciseMode) {
            preciseMode = false;
            return;
        }
        preciseMode = true;
    }

    //endregion

}
