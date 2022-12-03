package org.firstinspires.ftc.teamcode.auton;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.ArrayList;
import java.util.Locale;


// Credit: OpenFTC for a lot
@Config
@Autonomous
public class AutonAprilTagInitDetection extends LinearOpMode
{

    // Declare motors
    DcMotor motorFrontLeft;
    DcMotor motorBackLeft;
    DcMotor motorFrontRight;
    DcMotor motorBackRight;

    // Motor position variables
    private int flPos, blPos, frPos, brPos;

    // Operational Constants
    private final double fast = 0.7;
    private final double medium = 0.4;
    private final double slow = 0.2;
    public static double ticksPerInch = 114.6; // TODO: Verify this number
    public static double ticksPerDeg = 4; // TODO: Verify this number

    public static int distanceOne = 27;
    public static int distanceTwo = 26;

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

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

        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");

        // Adjust motor directions
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set motor run modes
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set motor target
        motorFrontLeft.setTargetPosition(0);
        motorBackLeft.setTargetPosition(0);
        motorFrontRight.setTargetPosition(0);
        motorBackRight.setTargetPosition(0);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        TelemetryPacket packet = new TelemetryPacket();
        FtcDashboard dashboard = FtcDashboard.getInstance();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

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
        if(tagOfInterest == null || tagOfInterest.id == LEFT){
            moveForward(26, medium);
            moveRight(-28, medium);
        }else if(tagOfInterest.id == MIDDLE){
            moveForward(26, medium);
        }else{
            moveForward(26, medium);
            moveRight(28, medium);
        }


        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        while (opModeIsActive()) {sleep(20);}
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



    private void moveForward(int distance, double speed) {
        // Distance is in inches. Negative amount moves backwards.

        // Fetch motor positions
        flPos = motorFrontLeft.getCurrentPosition();
        blPos = motorBackLeft.getCurrentPosition();
        frPos = motorFrontRight.getCurrentPosition();
        brPos = motorBackRight.getCurrentPosition();

        // Calculate new targets
        flPos += distance * ticksPerInch;
        blPos += distance * ticksPerInch;
        frPos += distance * ticksPerInch;
        brPos += distance * ticksPerInch;

        // Move robot to new position
        motorFrontLeft.setTargetPosition(flPos);
        motorBackLeft.setTargetPosition(blPos);
        motorFrontRight.setTargetPosition(frPos);
        motorBackRight.setTargetPosition(brPos);

        motorFrontLeft.setPower(speed);
        motorBackLeft.setPower(speed);
        motorFrontRight.setPower(speed);
        motorBackRight.setPower(speed);

        // Wait for movement to finish
        while (motorFrontLeft.isBusy() && motorBackLeft.isBusy() &&
                motorFrontRight.isBusy() && motorBackRight.isBusy()) {

            // Output some telemetry
            telemetry.addLine("Move Forward");
            telemetry.addData("Target", "fl: " + Integer.toString(flPos) + " bl: " +
                    Integer.toString(blPos) + " fr: " + Integer.toString(frPos) + " br: " +
                    Integer.toString(brPos));
            telemetry.addData("Actual",
                    "fl: " + Integer.toString(motorFrontLeft.getCurrentPosition()) +
                            " bl: " + Integer.toString(motorBackLeft.getCurrentPosition()) +
                            " fr: " + Integer.toString(motorFrontRight.getCurrentPosition()) +
                            " br: " + Integer.toString(motorBackRight.getCurrentPosition()));
            telemetry.update();
        }

        // After done, stop all motors
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
    }

    private void moveRight(int distance, double speed) {
        // distance is in inches. A negative distance moves left

        // Fetch motor positions
        flPos = motorFrontLeft.getCurrentPosition();
        blPos = motorBackLeft.getCurrentPosition();
        frPos = motorFrontRight.getCurrentPosition();
        brPos = motorBackRight.getCurrentPosition();

        // Calculate new targets
        flPos += distance * ticksPerInch;
        blPos -= distance * ticksPerInch;
        frPos -= distance * ticksPerInch;
        brPos += distance * ticksPerInch;

        // Move robot to new position
        motorFrontLeft.setTargetPosition(flPos);
        motorBackLeft.setTargetPosition(blPos);
        motorFrontRight.setTargetPosition(frPos);
        motorBackRight.setTargetPosition(brPos);

        motorFrontLeft.setPower(speed);
        motorBackLeft.setPower(speed);
        motorFrontRight.setPower(speed);
        motorBackRight.setPower(speed);

        // Wait for movement to finish
        while (motorFrontLeft.isBusy() && motorBackLeft.isBusy() &&
                motorFrontRight.isBusy() && motorBackRight.isBusy()) {

            // Output some telemetry
            telemetry.addLine("Strafe Right");
            telemetry.addData("Target", "fl: " + Integer.toString(flPos) + " bl: " +
                    Integer.toString(blPos) + " fr: " + Integer.toString(frPos) + " br: " +
                    Integer.toString(brPos));
            telemetry.addData("Actual",
                    "fl: " + Integer.toString(motorFrontLeft.getCurrentPosition()) +
                            " bl: " + Integer.toString(motorBackLeft.getCurrentPosition()) +
                            " fr: " + Integer.toString(motorFrontRight.getCurrentPosition()) +
                            " br: " + Integer.toString(motorBackRight.getCurrentPosition()));
            telemetry.update();
        }

        // After done, stop all motors
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
    }

    private void turnClockwise(int angle, double speed) {
        // angle is in degrees. Negative angle will turn counter clockwise

        // Fetch motor positions
        flPos = motorFrontLeft.getCurrentPosition();
        blPos = motorBackLeft.getCurrentPosition();
        frPos = motorFrontRight.getCurrentPosition();
        brPos = motorBackRight.getCurrentPosition();

        // Calculate new targets
        flPos += angle * ticksPerDeg;
        blPos += angle * ticksPerDeg;
        frPos -= angle * ticksPerDeg;
        brPos -= angle * ticksPerDeg;

        // Move robot to new position
        motorFrontLeft.setTargetPosition(flPos);
        motorBackLeft.setTargetPosition(blPos);
        motorFrontRight.setTargetPosition(frPos);
        motorBackRight.setTargetPosition(brPos);

        motorFrontLeft.setPower(speed);
        motorBackLeft.setPower(speed);
        motorFrontRight.setPower(speed);
        motorBackRight.setPower(speed);

        // Wait for movement to finish
        while (motorFrontLeft.isBusy() && motorBackLeft.isBusy() &&
                motorFrontRight.isBusy() && motorBackRight.isBusy()) {

            // Output some telemetry
            telemetry.addLine("Turn Clockwise");
            telemetry.addData("Target", "fl: " + Integer.toString(flPos) + " bl: " +
                    Integer.toString(blPos) + " fr: " + Integer.toString(frPos) + " br: " +
                    Integer.toString(brPos));
            telemetry.addData("Actual",
                    "fl: " + Integer.toString(motorFrontLeft.getCurrentPosition()) +
                            " bl: " + Integer.toString(motorBackLeft.getCurrentPosition()) +
                            " fr: " + Integer.toString(motorFrontRight.getCurrentPosition()) +
                            " br: " + Integer.toString(motorBackRight.getCurrentPosition()));
            telemetry.update();
        }

        // After done, stop all motors
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
    }
}