package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Lift {

    /*
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
     */

    private final Telemetry telemetry;
    private final CRServo liftMotor;
    private final TouchSensor bottom, top;
    private final Servo claw;

    final double DROP_IDLE = 0.85;
    final double DROP_DEPOSIT = 1.0;
    public double time = 1.0;
    private double liftSpeed = 0.5, DROP_TIME = time;

    ElapsedTime liftTimer = new ElapsedTime();

    public Lift(HardwareMap hardware, Telemetry givenTelemetry) {

        // Define Telemetry
        telemetry = givenTelemetry;

        // Define Motor
        liftMotor = hardware.crservo.get("vertical");

        // Touch Sensors
        bottom = hardware.touchSensor.get("bottom");
        top = hardware.touchSensor.get("top");

        // Claw
        claw = hardware.servo.get("manipulator");

        liftTimer.reset();

    }

    public enum LiftState {
        LIFT_START,
        LIFT_EXTEND,
        LIFT_DROP,
        LIFT_RETRACT,
        LIFT_END
    };

    public void cycle() {
        LiftState liftState = LiftState.LIFT_START;

        while(liftState != LiftState.LIFT_END) {
            switch (liftState) {
                case LIFT_START:
                    if (!atTop()) {
                        raise(liftSpeed);
                        telemetry.addData("Status", "RAISING");

                    } else {
                        liftState = Lift.LiftState.LIFT_EXTEND;
                    }
                    break;
                case LIFT_EXTEND:
                    if (atTop()) {
                        stop();
                        release();
                        telemetry.addData("Status", "RELEASING");

                        liftTimer.reset();
                        liftState = Lift.LiftState.LIFT_DROP;
                    }
                    break;
                case LIFT_DROP:
                    if (liftTimer.seconds() >= DROP_TIME) {
                        if (!atBottom()) {
                            lower(liftSpeed);
                            telemetry.addData("Status", "RETRACTING");
                        } else {
                            liftState = Lift.LiftState.LIFT_RETRACT;
                        }
                    }
                    break;
                case LIFT_RETRACT:
                    stop();
                    telemetry.addData("Status", "RETRACTED");
                    liftState = Lift.LiftState.LIFT_END;
                    break;
                case LIFT_END:
                    break;
            }
            telemetry.update();
        }
        return;
    }

    public void raise(double speed) {
        if (top.isPressed()) {
            liftMotor.setPower(-speed);
        }
    }

    public void lower(double speed) {
        if (bottom.isPressed()) {
            liftMotor.setPower(speed);
        }
    }

    public void stop() {
        liftMotor.setPower(0.0);
    }

    public void release() {
        claw.setPosition(0.9);
    }

    public void grab() {
        claw.setPosition(1.0);
    }

    public boolean atTop() {
        return (!top.isPressed());
    }

    public boolean atBottom() {
        return (!bottom.isPressed());
    }
}
