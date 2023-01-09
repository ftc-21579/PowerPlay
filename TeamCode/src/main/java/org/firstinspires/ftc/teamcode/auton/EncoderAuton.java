package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.util.EncoderMovement;

@Config
@Autonomous(name="TestingAuton", group="LinearOpmode")
public class EncoderAuton extends LinearOpMode {

    private EncoderMovement movement;
    private Lift lift;
    private double liftSpeed = 0.5, DROP_TIME = 2.0;
    public static double speed = 0.6;

    @Override
    public void runOpMode() {
        telemetry.setAutoClear(true);

        movement = new EncoderMovement(hardwareMap, telemetry);
        lift = new Lift(hardwareMap, telemetry);
        Lift.LiftState liftState = Lift.LiftState.LIFT_START;

        waitForStart();

        boolean cycled = false;
        boolean movedForward1 = false, turned1 = false, movedForward2 = false, turned2 = false, movedBackward = false;

        while(opModeIsActive()) {


            if (!movedForward1) {
                movement.moveForward(50, speed);
                movedForward1 = true;
            } else if (movedForward1 && !turned1) {
                movement.turnClockwise(43, speed);
                turned1 = true;
            } else if (movedForward1 && turned1 && !movedForward2) {
                movement.moveForward(2, speed);
                movedForward2 = true;
            }   else if (movedForward1 && turned1 && movedForward2 && !cycled) {
                lift.cycle();
                cycled = true;
            } else if (movedForward1 && turned1 && movedForward2 && cycled && !turned2) {
                movement.turnCounterClockwise(40, speed);
                turned2 = true;
            } else if (movedForward1 && turned1 && movedForward2 && cycled && turned2 && !movedBackward) {
                movement.moveForward(-26, speed);
                movedBackward = true;
            }




        }

        //movement.moveForward(24, 0.4);
    }
}

