package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.util.EncoderMovement;

@Config
@Autonomous(name="TestingAuton", group="LinearOpmode")
public class EncoderAuton extends LinearOpMode {

    private EncoderMovement movement;
    private Lift lift;
    private double liftSpeed = 0.5, DROP_TIME = 2.0;

    @Override
    public void runOpMode() {
        telemetry.setAutoClear(true);

        movement = new EncoderMovement(hardwareMap, telemetry);
        lift = new Lift(hardwareMap, telemetry);
        Lift.LiftState liftState = Lift.LiftState.LIFT_START;

        ElapsedTime liftTimer = new ElapsedTime();
        liftTimer.reset();

        waitForStart();

        while(opModeIsActive()) {
            switch (liftState) {
                case LIFT_START:
                    if (!lift.atTop()) {
                        lift.raise(liftSpeed);
                        telemetry.addData("Status: ","RAISING");

                    } else {
                        liftState = Lift.LiftState.LIFT_EXTEND;
                    }
                    break;
                case LIFT_EXTEND:
                    if (lift.atTop()) {
                        lift.release();
                        telemetry.addData("Status: ","RELEASING");

                        liftTimer.reset();
                        liftState = Lift.LiftState.LIFT_DROP;
                    }
                    break;
                case LIFT_DROP:
                    if (liftTimer.seconds() >= DROP_TIME) {
                        if (!lift.atBottom()) {
                            lift.lower(liftSpeed);
                            telemetry.addData("Status: ","RETRACTING");
                        } else {
                            liftState = Lift.LiftState.LIFT_RETRACT;
                        }
                    }
                    break;
                case LIFT_RETRACT:
                    telemetry.addData("Status: ","RETRACTED");
                    break;
            }
            telemetry.update();
        }

        //movement.moveForward(24, 0.4);
        //lift.maxHeight();
        //lift.openClaw();
    }
}

