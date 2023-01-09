package org.firstinspires.ftc.teamcode.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="MainDrive")
public class RobotCentricMecanum extends LinearOpMode {

    double vertPow, gripPos;
    boolean topPressed, bottomPressed = false;
    double MIN_POSITION = 0.9, MAX_POSITION = 1;

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
        // Reverse left motors if you are using NeveRests
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addLine("Ready");
        telemetry.update();

        gripPos = 0.5;
        vertPow = 0;


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            double max = 1.0;

            Boolean rBump = gamepad2.right_bumper;
            Boolean lBump = gamepad2.left_bumper;

            double rTrigger = gamepad2.right_trigger;
            double lTrigger = gamepad2.left_trigger;

            bottomPressed = bottom.isPressed();
            topPressed = top.isPressed();

            // Gripper
            double gripPos = gripServo.getPosition();
            if (rBump) {
                gripPos -= 0.01;
            }
            if (lBump) {
                gripPos += 0.01;
            }


            telemetry.addData("botton", bottom.isPressed());
            // Vertical Slides

            if (rTrigger > 0.2 && bottom.isPressed()) {
                vertPow = 1.0;
            }
            else if (lTrigger > 0.2 && top.isPressed()) {
                vertPow = -1.0;
            }
            else {
                vertPow = 0.0;
            }

            gripServo.setPosition(Range.clip(gripPos, MIN_POSITION, MAX_POSITION));
            vertServo1.setPower(vertPow);
            telemetry.addData("gripPos", gripPos);
            telemetry.addData("vertPow", vertPow);

            /* Doesnt Work
            if (gamepad1.left_trigger > 0.2 || gamepad1.right_trigger > 0.2) {
                max = 0.5;
            } else {
                max = 1.0;
            }
            */

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), max);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            packet.put("frontLeftPower", frontLeftPower);
            packet.put("frontRightPower", frontRightPower);
            packet.put("backLeftPower", backLeftPower);
            packet.put("backRightPower", backRightPower);

            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);

            dashboard.sendTelemetryPacket(packet);
        }
    }
}
