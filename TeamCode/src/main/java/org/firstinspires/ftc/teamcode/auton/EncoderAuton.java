package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.PIDController;

@Config
@Autonomous(name="TestingAuton", group="LinearOpmode")
@Disabled
public class EncoderAuton extends LinearOpMode {

    //private EncoderMovement movement;
    //private double liftSpeed = 0.5, DROP_TIME = 2.0;
    //public static double speed = 0.4;
    public static double Kp = 0.01, Ki = 0, Kd = 0;
    public static int targetInches = 24;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    DcMotorEx liftEncoder;
    CRServo liftMotor;

    PIDController control = new PIDController(Kp, Ki, Kd, dashboardTelemetry);

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

        while(opModeIsActive()) {
            int targetPosition = (int)(targetInches * 64.68056888);

            // Update pid controller
            double command = control.update(targetPosition, liftEncoder.getCurrentPosition());
            command = Range.clip(command, -1, 1);
            // Assign PID output
            dashboardTelemetry.addData("Command", command);
            liftMotor.setPower(command);

            dashboardTelemetry.update();
            //telemetry.update();
        }
    }
}

