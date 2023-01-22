package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Allows the lift to extend/retract and handle cones
 */
@Config
public class Lift {

    //region Variable Init
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

    //endregion

    //region Public Methods

    /**
     * Will cycle the lift (Extend, release, retract)
     * This method will run immediately
     */
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

    /**
     * Will enable the raising of the lift
     *
     * @param speed The speed at which the motor should spin (0.0 - 1.0)
     */
    public void raise(double speed) {
        if (top.isPressed()) {
            liftMotor.setPower(-speed);
        }
    }

    /**
     * Will enable the lowering of the lift
     *
     * @param speed The speed at which the motor should spin (0.0 - 1.0)
     */
    public void lower(double speed) {
        if (bottom.isPressed()) {
            liftMotor.setPower(speed);
        }
    }

    /**
     * Will force stop the movement of the lift
     */
    public void stop() {
        liftMotor.setPower(0.0);
    }

    /**
     * Will release the held cone (if there is one)
     */
    public void release() {
        claw.setPosition(0.9);
    }

    /**
     * Will grab a cone/close claw
     */
    public void grab() {
        claw.setPosition(1.0);
    }

    /**
     * Will return whether the lift is at the top
     */
    public boolean atTop() {
        return (!top.isPressed());
    }

    /**
     * Will return whether the lift is at the top
     */
    public boolean atBottom() {
        return (!bottom.isPressed());
    }

    //endregion
}
