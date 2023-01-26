package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


import org.firstinspires.ftc.teamcode.util.PIDController;

@Config
@Autonomous(name="TestingAuton", group="LinearOpmode")
public class EncoderAuton extends LinearOpMode {

    //private EncoderMovement movement;
    //private double liftSpeed = 0.5, DROP_TIME = 2.0;
    //public static double speed = 0.4;
    public static double Kp, Ki, Kd;
    public static int targetInches = 24;

    DcMotorEx liftEncoder;
    CRServo liftMotor;

    PIDController control = new PIDController(Kp, Ki, Kd, telemetry);

    @Override
    public void runOpMode() {
        telemetry.setAutoClear(true);

        liftEncoder = hardwareMap.get(DcMotorEx.class, "motorFrontRight");
        liftMotor = hardwareMap.crservo.get("vertical"); // Ensure Spark Mini is on Braking

        liftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        //movement = new EncoderMovement(hardwareMap, telemetry);
        //Lift.LiftState liftState = Lift.LiftState.LIFT_START;

        waitForStart();

        int targetPosition = (int)(targetInches * 64.68056888);

        while(opModeIsActive()) {
            // Update pid controller
            double command = control.update(targetPosition, liftEncoder.getCurrentPosition());
            // Assign PID output
            telemetry.addData("Command", command);
            liftMotor.setPower(command);

            telemetry.update();
        }
    }
}

