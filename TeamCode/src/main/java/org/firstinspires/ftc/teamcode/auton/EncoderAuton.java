package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.newLift;
import org.firstinspires.ftc.teamcode.util.EncoderMovement;

@Config
@Autonomous(name="TestingAuton", group="LinearOpmode")
public class EncoderAuton extends LinearOpMode {

    private EncoderMovement movement;
    private newLift lift;
    private double liftSpeed = 0.5, DROP_TIME = 2.0;
    public static double speed = 0.4;

    @Override
    public void runOpMode() {
        telemetry.setAutoClear(true);

        movement = new EncoderMovement(hardwareMap, telemetry);
        lift = new newLift(hardwareMap, telemetry);
        //Lift.LiftState liftState = Lift.LiftState.LIFT_START;

        waitForStart();

        boolean cycled = false;
        boolean movedForward1 = false, turned1 = false, movedForward2 = false, turned2 = false, movedBackward = false;

        //lift.grab();

        lift.up(5, 0.5);

        //movement.moveForward(24, 0.4);
    }
}

