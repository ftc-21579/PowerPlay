package org.firstinspires.ftc.teamcode.auton;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.PIDController;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicBoolean;


// Credit: OpenFTC for a lot
@Config
@Autonomous(name="1+4 Left")
@Disabled
public class AutonomousTesting extends LinearOpMode
{
    public static double Kp = 0.005, Ki = 0, Kd = 0;
    public static int targetInches = 0;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    DcMotorEx liftEncoder;
    CRServo liftMotor;

    PIDController control = new PIDController(Kp, Ki, Kd, dashboardTelemetry);

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    PoleObserverPipeline poleObserverPipeline;

    int strafeDistance = 0;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 1,2,3 from the 36h11 family
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode()
    {
        telemetry.setAutoClear(true);
        TelemetryPacket packet = new TelemetryPacket();
        FtcDashboard dashboard = FtcDashboard.getInstance();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-36, -64, Math.toRadians(90)); // Set start pose to center of the field, facing north
        drive.setPoseEstimate(startPose);

        Pose2d scorePose = new Pose2d(-35, -10, Math.toRadians(35));

        liftEncoder = hardwareMap.get(DcMotorEx.class, "motorFrontRight");
        liftMotor = hardwareMap.crservo.get("vertical"); // Ensure Spark Mini is on Braking

        Servo guide = hardwareMap.servo.get("guide");
        Servo gripServo = hardwareMap.servo.get("manipulator");
        Servo signal = hardwareMap.servo.get("signal");

        liftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        poleObserverPipeline = new PoleObserverPipeline(telemetry);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(1280,720, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard.getInstance().startCameraStream(camera,5);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        TrajectorySequence parkingOne = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(0.5, () -> {
                    signal.setPosition(0.0);
                })
                .addTemporalMarker(() -> {
                    targetInches = 0;
                    signal.setPosition(0.5);
                    gripServo.setPosition(1.0);
                })
                .waitSeconds(0.35)
                .addDisplacementMarker(() -> {
                    targetInches = 37;
                })
                .splineTo(new Vector2d(-36, -27), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(-34.5, -12.5, Math.toRadians(45)), Math.toRadians(60))
                .waitSeconds(0.3)
                .addTemporalMarker(() -> {
                    gripServo.setPosition(0.75);
                    targetInches = 7;
                    signal.setPosition(0.55);
                })
                .lineToSplineHeading(new Pose2d(-40, -14, Math.toRadians(180)))
                .splineToLinearHeading(new Pose2d(-59.5, -14, Math.toRadians(180)), Math.toRadians(180))
               .addTemporalMarker(() -> {
                    gripServo.setPosition(1.0);
                })
                .waitSeconds(0.225)
                .addTemporalMarker(() -> {
                    targetInches = 37;
                })
                .back(19)
                .splineToSplineHeading(new Pose2d(-34.5, -12.5, Math.toRadians(45)), Math.toRadians(55))
                .waitSeconds(0.3)
                .addTemporalMarker(() -> {
                    gripServo.setPosition(0.75);
                    targetInches = 5;
                })
                .lineToSplineHeading(new Pose2d(-40, -14, Math.toRadians(180)))
                .splineToLinearHeading(new Pose2d(-59.5, -14, Math.toRadians(180)), Math.toRadians(180))
                .addTemporalMarker(() -> {
                    gripServo.setPosition(1.0);
                })
                .waitSeconds(0.225)
                .addTemporalMarker(() -> {
                    targetInches = 37;
                })
                .back(19)
                .splineToSplineHeading(new Pose2d(-34.5, -12.5, Math.toRadians(45)), Math.toRadians(55))
                .waitSeconds(0.3)
                .addTemporalMarker(() -> {
                    gripServo.setPosition(0.75);
                    targetInches = 4;
                })
                .lineToSplineHeading(new Pose2d(-40, -14, Math.toRadians(180)))
                .splineToLinearHeading(new Pose2d(-59.5, -14, Math.toRadians(180)), Math.toRadians(180))
                .addTemporalMarker(() -> {
                    gripServo.setPosition(1.0);
                })
                .waitSeconds(0.225)
                .addTemporalMarker(() -> {
                    targetInches = 37;
                })
                .back(19)
                .splineToSplineHeading(new Pose2d(-34.5, -12.5, Math.toRadians(45)), Math.toRadians(55))
                .waitSeconds(0.3)
                .addTemporalMarker(() -> {
                    gripServo.setPosition(0.75);
                    targetInches = 3;
                })
                .lineToSplineHeading(new Pose2d(-40, -14, Math.toRadians(180)))
                .splineToLinearHeading(new Pose2d(-59.5, -14, Math.toRadians(180)), Math.toRadians(180))
                .addTemporalMarker(() -> {
                    gripServo.setPosition(1.0);
                })
                .waitSeconds(0.225)
                .addTemporalMarker(() -> {
                    targetInches = 37;
                })
                .back(19)
                .splineToSplineHeading(new Pose2d(-34.5, -12.5, Math.toRadians(45)), Math.toRadians(55))
                .waitSeconds(0.3)
                .addTemporalMarker(() -> {
                    gripServo.setPosition(0.9);
                    targetInches = 0;
                })
                .splineToLinearHeading(new Pose2d(-36, -14, Math.toRadians(90)), Math.toRadians(90))
                .strafeLeft(24)
                .build();

        TrajectorySequence parkingTwo = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(0.5, () -> {
                    signal.setPosition(0.0);
                })
                .addTemporalMarker(() -> {
                    targetInches = 0;
                    signal.setPosition(0.5);
                    gripServo.setPosition(1.0);
                })
                .waitSeconds(0.35)
                .addDisplacementMarker(() -> {
                    targetInches = 37;
                })
                .splineTo(new Vector2d(-36, -27), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(-34.5, -12.5, Math.toRadians(45)), Math.toRadians(60))
                .waitSeconds(0.3)
                .addTemporalMarker(() -> {
                    gripServo.setPosition(0.75);
                    targetInches = 7;
                    signal.setPosition(0.55);
                })
                .lineToSplineHeading(new Pose2d(-40, -14, Math.toRadians(180)))
                .splineToLinearHeading(new Pose2d(-59.5, -14, Math.toRadians(180)), Math.toRadians(180))
                .addTemporalMarker(() -> {
                    gripServo.setPosition(1.0);
                })
                .waitSeconds(0.225)
                .addTemporalMarker(() -> {
                    targetInches = 37;
                })
                .back(19)
                .splineToSplineHeading(new Pose2d(-34.5, -12.5, Math.toRadians(45)), Math.toRadians(55))
                .waitSeconds(0.3)
                .addTemporalMarker(() -> {
                    gripServo.setPosition(0.75);
                    targetInches = 5;
                })
                .lineToSplineHeading(new Pose2d(-40, -14, Math.toRadians(180)))
                .splineToLinearHeading(new Pose2d(-59.5, -14, Math.toRadians(180)), Math.toRadians(180))
                .addTemporalMarker(() -> {
                    gripServo.setPosition(1.0);
                })
                .waitSeconds(0.225)
                .addTemporalMarker(() -> {
                    targetInches = 37;
                })
                .back(19)
                .splineToSplineHeading(new Pose2d(-34.5, -12.5, Math.toRadians(45)), Math.toRadians(55))
                .waitSeconds(0.3)
                .addTemporalMarker(() -> {
                    gripServo.setPosition(0.75);
                    targetInches = 4;
                })
                .lineToSplineHeading(new Pose2d(-40, -14, Math.toRadians(180)))
                .splineToLinearHeading(new Pose2d(-59.5, -14, Math.toRadians(180)), Math.toRadians(180))
                .addTemporalMarker(() -> {
                    gripServo.setPosition(1.0);
                })
                .waitSeconds(0.225)
                .addTemporalMarker(() -> {
                    targetInches = 37;
                })
                .back(19)
                .splineToSplineHeading(new Pose2d(-34.5, -12.5, Math.toRadians(45)), Math.toRadians(55))
                .waitSeconds(0.3)
                .addTemporalMarker(() -> {
                    gripServo.setPosition(0.75);
                    targetInches = 3;
                })
                .lineToSplineHeading(new Pose2d(-40, -14, Math.toRadians(180)))
                .splineToLinearHeading(new Pose2d(-59.5, -14, Math.toRadians(180)), Math.toRadians(180))
                .addTemporalMarker(() -> {
                    gripServo.setPosition(1.0);
                })
                .waitSeconds(0.225)
                .addTemporalMarker(() -> {
                    targetInches = 37;
                })
                .back(19)
                .splineToSplineHeading(new Pose2d(-34.5, -12.5, Math.toRadians(45)), Math.toRadians(55))
                .waitSeconds(0.3)
                .addTemporalMarker(() -> {
                    gripServo.setPosition(0.9);
                    targetInches = 0;
                })
                .splineToLinearHeading(new Pose2d(-36, -14, Math.toRadians(90)), Math.toRadians(90))
                .build();

        TrajectorySequence parkingThree = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(0.5, () -> {
                    signal.setPosition(0.0);
                })
                .addTemporalMarker(() -> {
                    targetInches = 0;
                    signal.setPosition(0.5);
                    gripServo.setPosition(1.0);
                })
                .waitSeconds(0.35)
                .addDisplacementMarker(() -> {
                    targetInches = 37;
                })
                .splineTo(new Vector2d(-36, -27), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(-34.5, -12.5, Math.toRadians(45)), Math.toRadians(60))
                .waitSeconds(0.3)
                .addTemporalMarker(() -> {
                    gripServo.setPosition(0.75);
                    targetInches = 7;
                    signal.setPosition(0.55);
                })
                .lineToSplineHeading(new Pose2d(-40, -14, Math.toRadians(180)))
                .splineToLinearHeading(new Pose2d(-59.5, -14, Math.toRadians(180)), Math.toRadians(180))
                .addTemporalMarker(() -> {
                    gripServo.setPosition(1.0);
                })
                .waitSeconds(0.225)
                .addTemporalMarker(() -> {
                    targetInches = 37;
                })
                .back(19)
                .splineToSplineHeading(new Pose2d(-34.5, -12.5, Math.toRadians(45)), Math.toRadians(55))
                .waitSeconds(0.3)
                .addTemporalMarker(() -> {
                    gripServo.setPosition(0.75);
                    targetInches = 5;
                })
                .lineToSplineHeading(new Pose2d(-40, -14, Math.toRadians(180)))
                .splineToLinearHeading(new Pose2d(-59.5, -14, Math.toRadians(180)), Math.toRadians(180))
                .addTemporalMarker(() -> {
                    gripServo.setPosition(1.0);
                })
                .waitSeconds(0.225)
                .addTemporalMarker(() -> {
                    targetInches = 37;
                })
                .back(19)
                .splineToSplineHeading(new Pose2d(-34.5, -12.5, Math.toRadians(45)), Math.toRadians(55))
                .waitSeconds(0.3)
                .addTemporalMarker(() -> {
                    gripServo.setPosition(0.75);
                    targetInches = 4;
                })
                .lineToSplineHeading(new Pose2d(-40, -14, Math.toRadians(180)))
                .splineToLinearHeading(new Pose2d(-59.5, -14, Math.toRadians(180)), Math.toRadians(180))
                .addTemporalMarker(() -> {
                    gripServo.setPosition(1.0);
                })
                .waitSeconds(0.225)
                .addTemporalMarker(() -> {
                    targetInches = 37;
                })
                .back(19)
                .splineToSplineHeading(new Pose2d(-34.5, -12.5, Math.toRadians(45)), Math.toRadians(55))
                .waitSeconds(0.3)
                .addTemporalMarker(() -> {
                    gripServo.setPosition(0.75);
                    targetInches = 3;
                })
                .lineToSplineHeading(new Pose2d(-40, -14, Math.toRadians(180)))
                .splineToLinearHeading(new Pose2d(-59.5, -14, Math.toRadians(180)), Math.toRadians(180))
                .addTemporalMarker(() -> {
                    gripServo.setPosition(1.0);
                })
                .waitSeconds(0.225)
                .addTemporalMarker(() -> {
                    targetInches = 37;
                })
                .back(19)
                .splineToSplineHeading(new Pose2d(-34.5, -12.5, Math.toRadians(45)), Math.toRadians(55))
                .waitSeconds(0.3)
                .addTemporalMarker(() -> {
                    gripServo.setPosition(0.9);
                    targetInches = 0;
                })
                .splineToLinearHeading(new Pose2d(-36, -14, Math.toRadians(90)), Math.toRadians(90))
                .strafeRight(24)
                .build();

        //drive.followTrajectorySequenceAsync(trajSeq);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                    tagToPacket(tagOfInterest, packet);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            dashboard.sendTelemetryPacket(packet);

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */
        camera.setPipeline(poleObserverPipeline);

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful */
        //region MOVEMENT
        // speed 0.4 is pretty good

        if(tagOfInterest == null || tagOfInterest.id == LEFT){
            drive.followTrajectorySequenceAsync(parkingOne);
        }else if(tagOfInterest.id == MIDDLE){
            drive.followTrajectorySequenceAsync(parkingTwo);
        }else{
            drive.followTrajectorySequenceAsync(parkingThree);
        }

        while(opModeIsActive()) {
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
        //endregion


        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        //while (opModeIsActive()) {sleep(20);}
    }

    @SuppressLint("DefaultLocale")
    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

    @SuppressLint("DefaultLocale")
    void tagToPacket(AprilTagDetection detection, TelemetryPacket packet)
    {
        packet.put("Detected Tag", String.format("Detected tag ID=%d", detection.id));
        packet.put("Translation X", String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        packet.put("Translation Y", String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        packet.put("Translation Z", String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
    }

}
