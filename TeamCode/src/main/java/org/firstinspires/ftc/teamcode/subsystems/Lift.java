package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Lift {

    //region Variable Init
    private final Telemetry telemetry;
    private final CRServo liftMotor;
    private final TouchSensor bottom, top;
    private final Servo claw;

    private final double MIN_POSITION = 0.8, MAX_POSITION = 1;


    public Lift(HardwareMap hardware, Telemetry givenTelemetry) {

        // Define Telemetry
        telemetry = givenTelemetry;

        // Define Motor
        liftMotor = hardware.crservo.get("vertical");

        // Touch Sensors
        bottom = hardware.touchSensor.get("bottom");
        top = hardware.touchSensor.get("top");

        // Claw
        claw = hardware.servo.get("gripper");
    }

    //endregion

    //region Public Methods
    public void maxHeight () {
        while (top.isPressed()) {
            liftMotor.setPower(-0.5);
        }
    }

    public void minHeight() {
        while(bottom.isPressed()) {
            liftMotor.setPower(0.5);
        }
    }

    public void openClaw() {
        claw.setPosition(MIN_POSITION);
    }

    public void closeClaw() {
        claw.setPosition(MAX_POSITION);
    }

}
