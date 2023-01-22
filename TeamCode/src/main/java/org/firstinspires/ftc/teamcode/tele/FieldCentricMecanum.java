package org.firstinspires.ftc.teamcode.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="FieldCentric")
public class FieldCentricMecanum extends LinearOpMode {

    double vertPow, gripPos;
    boolean topPressed, bottomPressed = false;
    double MIN_POSITION = 0.9, MAX_POSITION = 1;
    double multiplier = 1.0;

    @Override
    public void runOpMode() throws InterruptedException {

        // FTC Dashboard Setups
        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();

        // Manipulator Servo
        Servo gripServo = hardwareMap.servo.get("manipulator");

        // Vertical Servo
        CRServo vertServo1 = hardwareMap.crservo.get("vertical");

        // Touch Sensors
        TouchSensor bottom = hardwareMap.touchSensor.get("bottom");
        TouchSensor top = hardwareMap.touchSensor.get("top");

        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("motorBackRight");

        // Reverse the right side motors
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);

        telemetry.addLine("Ready");
        telemetry.update();

        gripPos = 0.5;
        vertPow = 0;

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if(gamepad1.left_trigger > 0.2 || gamepad1.right_trigger > 0.2 || gamepad1.left_bumper || gamepad1.right_bumper) {
                multiplier = 0.5;
            }
            else {
                multiplier = 1.0;
            }

            // Gamepad Inputs
            double y = -gamepad1.left_stick_y * multiplier; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1 * multiplier; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x * 0.6;

            Boolean rBump = gamepad2.right_bumper;
            Boolean lBump = gamepad2.left_bumper;

            double rTrigger = gamepad2.right_trigger;
            double lTrigger = gamepad2.left_trigger;

            bottomPressed = bottom.isPressed();
            topPressed = top.isPressed();


            // Gripper
            if (gamepad2.a || gamepad2.x) {
                gripPos -= 0.1;
            }
            if (gamepad2.b || gamepad2.y) {
                gripPos += 0.1;
            }
            gripPos = Range.clip(gripPos, MIN_POSITION, MAX_POSITION);


            // Vertical Slides
            if (rTrigger > 0.2 && bottom.isPressed()) {
                vertPow = 1.0;
            }
            else if (rBump && top.isPressed()) {
                vertPow = -1.0;
            }
            else if (lTrigger > 0.2 && bottom.isPressed()) {
                vertPow = 0.5;
            }
            else if (lBump && top.isPressed()) {
                vertPow = -0.5;
            }
            else {
                vertPow = 0;
            }

            gripServo.setPosition(Range.clip(gripPos, MIN_POSITION, MAX_POSITION));
            vertServo1.setPower(vertPow);
            telemetry.addData("gripPos", gripPos);
            telemetry.addData("vertPow", vertPow);

            // Read inverse IMU heading, as the IMU heading is CW positive
            double botHeading = -imu.getAngularOrientation().firstAngle;

            double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
            double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            packet.put("frontLeftPower", frontLeftPower);
            packet.put("frontRightPower", frontRightPower);
            packet.put("backLeftPower", backLeftPower);
            packet.put("backRightPower", backRightPower);

            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);
        }
    }
}