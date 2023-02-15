package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.PIDController;

@Autonomous(name="BackBeforeDropTest")
public class rrTestingAutonBACKBEFOREDROP extends LinearOpMode {

    public static double Kp = 0.01, Ki = 0, Kd = 0;
    public static int targetInches = 0;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    DcMotorEx liftEncoder;
    CRServo liftMotor;

    PIDController control = new PIDController(Kp, Ki, Kd, dashboardTelemetry);

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-35, -62, Math.toRadians(90)); // Set start pose to center of the field, facing north
        drive.setPoseEstimate(startPose);

        Pose2d scorePose = new Pose2d(-35, -10, Math.toRadians(35));

        liftEncoder = hardwareMap.get(DcMotorEx.class, "motorFrontRight");
        liftMotor = hardwareMap.crservo.get("vertical"); // Ensure Spark Mini is on Braking

        Servo guide = hardwareMap.servo.get("guide");
        Servo gripServo = hardwareMap.servo.get("manipulator");


        liftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(() -> {
                    gripServo.setPosition(1.0);
                    targetInches = 37;

                })
                .splineToSplineHeading(new Pose2d(-35, -10, Math.toRadians(45)), Math.toRadians(85))
                .addDisplacementMarker(() -> {
                    guide.setPosition(0.0);
                })
                .waitSeconds(0.25)

                .addDisplacementMarker(() -> {
                    gripServo.setPosition(0.9);
                })
                .waitSeconds(0.25)
                .addDisplacementMarker(() -> {
                    guide.setPosition(0.33);
                    targetInches = 7;
                })
                .splineToLinearHeading(new Pose2d(-40, -12, Math.toRadians(180)), Math.toRadians(180))
                .forward(20)
                .waitSeconds(0.25)
                .addDisplacementMarker(() -> {
                    gripServo.setPosition(1.0);
                })
                .waitSeconds(0.25)
                .addDisplacementMarker(() -> {
                    targetInches = 37;
                })
                .back(20)
                .splineToSplineHeading(new Pose2d(-35, -10, Math.toRadians(45)), Math.toRadians(45))
                .addDisplacementMarker(() -> {
                    guide.setPosition(0.0);
                })
                .waitSeconds(0.25)

                .addDisplacementMarker(() -> {
                    gripServo.setPosition(0.9);
                })
                .waitSeconds(0.25)
                .addDisplacementMarker(() -> {
                    guide.setPosition(0.33);
                    targetInches = 7;
                })
                .splineToLinearHeading(new Pose2d(-40, -12, Math.toRadians(180)), Math.toRadians(180))
                .forward(20)
                .waitSeconds(0.25)
                .addDisplacementMarker(() -> {
                    gripServo.setPosition(1.0);
                })
                .waitSeconds(0.25)
                .addDisplacementMarker(() -> {
                    targetInches = 37;
                })
                .back(20)
                .splineToSplineHeading(new Pose2d(-35, -10, Math.toRadians(45)), Math.toRadians(45))
                .addDisplacementMarker(() -> {
                    guide.setPosition(0.0);
                })
                .waitSeconds(0.25)

                .addDisplacementMarker(() -> {
                    gripServo.setPosition(0.9);
                })
                .waitSeconds(0.25)
                .addDisplacementMarker(() -> {
                    guide.setPosition(0.33);
                    targetInches = 7;
                })
                .splineToLinearHeading(new Pose2d(-40, -12, Math.toRadians(180)), Math.toRadians(180))
                .forward(20)
                .waitSeconds(0.25)
                .addDisplacementMarker(() -> {
                    gripServo.setPosition(1.0);
                })
                .waitSeconds(0.25)
                .addDisplacementMarker(() -> {
                    targetInches = 37;
                })
                .back(20)
                .splineToSplineHeading(new Pose2d(-35, -10, Math.toRadians(45)), Math.toRadians(45))
                .addDisplacementMarker(() -> {
                    guide.setPosition(0.0);
                })
                .waitSeconds(0.25)

                .addDisplacementMarker(() -> {
                    gripServo.setPosition(0.9);
                })
                .waitSeconds(0.25)
                .addDisplacementMarker(() -> {
                    guide.setPosition(0.33);
                    targetInches = 7;
                })
                .splineToLinearHeading(new Pose2d(-40, -12, Math.toRadians(180)), Math.toRadians(180))
                .forward(20)
                .waitSeconds(0.25)
                .addDisplacementMarker(() -> {
                    gripServo.setPosition(1.0);
                })
                .waitSeconds(0.25)
                .addDisplacementMarker(() -> {
                    targetInches = 37;
                })
                .back(20)
                .splineToSplineHeading(new Pose2d(-35, -10, Math.toRadians(45)), Math.toRadians(45))
                .addDisplacementMarker(() -> {
                    guide.setPosition(0.0);
                })
                .waitSeconds(0.25)

                .addDisplacementMarker(() -> {
                    gripServo.setPosition(0.9);
                })
                .waitSeconds(0.25)
                .addDisplacementMarker(() -> {
                    guide.setPosition(0.33);
                    targetInches = 7;
                })
                .splineToLinearHeading(new Pose2d(-40, -12, Math.toRadians(180)), Math.toRadians(180))
                .forward(20)
                .waitSeconds(0.25)
                .addDisplacementMarker(() -> {
                    gripServo.setPosition(1.0);
                })
                .waitSeconds(0.25)
                .addDisplacementMarker(() -> {
                    targetInches = 37;
                })
                .back(20)
                .splineToLinearHeading(new Pose2d(-36, -12, Math.toRadians(90)), Math.toRadians(90))
                .strafeLeft(24)
                .build();

        drive.followTrajectorySequenceAsync(trajSeq);

        waitForStart();

        while (opModeIsActive()) {
            drive.update();

            int targetPosition = (int) (targetInches * 64.68056888);
            // Update pid controller
            double command = control.update(targetPosition, liftEncoder.getCurrentPosition());
            command = Range.clip(command, -1, 1);
            // Assign PID output
            dashboardTelemetry.addData("Command", command);
            liftMotor.setPower(command);

            dashboardTelemetry.update();
        }
    }
}