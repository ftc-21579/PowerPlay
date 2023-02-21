package org.firstinspires.ftc.teamcode.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.PIDController;

// Warning: this code is garbage and nobody knows how it works :D
@TeleOp(name="RobotCentric")
public class RobotCentricMecanum extends LinearOpMode {

    double vertPow, gripPos;
    boolean topPressed, bottomPressed = false;
    double MIN_POSITION = 0.9, MAX_POSITION = 1;
    double multiplier = 1.0;

    public static double Kp = 0.005, Ki = 0, Kd = 0;
    public static int targetInches = 0;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    DcMotorEx liftEncoder;
    CRServo liftMotor;

    PIDController control = new PIDController(Kp, Ki, Kd, dashboardTelemetry);

    @Override
    public void runOpMode() throws InterruptedException {

        // FTC Dashboard Setups
        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();

        // Servo
        Servo gripServo = hardwareMap.servo.get("manipulator");
        Servo guide = hardwareMap.servo.get("guide");

        liftEncoder = hardwareMap.get(DcMotorEx.class, "motorFrontRight");
        liftMotor = hardwareMap.crservo.get("vertical"); // Ensure Spark Mini is on Braking

        //liftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //liftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

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

            double y = -gamepad1.left_stick_y * multiplier; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1 * multiplier; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x * 0.6;

            Boolean manual = false;

            Boolean rBump = gamepad2.right_bumper;
            Boolean lBump = gamepad2.left_bumper;

            double rTrigger = gamepad2.right_trigger;
            double lTrigger = gamepad2.left_trigger;

            bottomPressed = bottom.isPressed();
            topPressed = top.isPressed();


            // Gripper
            if (gamepad2.a) {
                gripServo.setPosition(0.9);
                Thread.sleep(500);
                guide.setPosition(0.33);
            }
            if (gamepad2.b) {
                gripServo.setPosition(1.0);
            }
            gripPos = Range.clip(gripPos, MIN_POSITION, MAX_POSITION);


            // Guide
            if (gamepad2.x) {
                guide.setPosition(0.33);
                telemetry.addData("Guide Pos", 0.33);
            } else if (gamepad2.y) {
                guide.setPosition(0.0);
                telemetry.addData("Guide Pos", 0.0);
            }

            // Auto heights
            if (gamepad2.dpad_up) {
                targetInches = 38;
            } else if (gamepad2.dpad_right) {
                targetInches = 28;
            } else if (gamepad2.dpad_down) {
                targetInches = 18;
            } else if (gamepad2.dpad_left) {
                targetInches = 1;
            }

            if (gamepad2.guide) {
                if (manual) {
                    manual = false;
                }
                else {
                    manual = true;
                }
            }

            // Vertical Slides
            if (manual) {
                if (rTrigger > 0.2 && bottom.isPressed()) {
                    vertPow = -1.0;
                }
                else if (rBump && top.isPressed()) {
                    vertPow = 1.0;
                }
                else if (lTrigger > 0.2 && bottom.isPressed()) {
                    vertPow = -0.5;
                }
                else if (lBump) {
                    vertPow = 0.5;
                }
                else {
                    vertPow = 0.0;
                }
                int targetPosition = (int)(targetInches * 64.68056888);
                control.update(targetPosition, liftEncoder.getCurrentPosition());

                liftMotor.setPower(vertPow);
            }
            else {
                int targetPosition = (int)(targetInches * 64.68056888);
                // Update pid controller
                double command = control.update(targetPosition, liftEncoder.getCurrentPosition());
                command = Range.clip(command, -1, 1);
                // Assign PID output
                dashboardTelemetry.addData("Command", command);
                liftMotor.setPower(command);
            }

            //gripServo.setPosition(Range.clip(gripPos, MIN_POSITION, MAX_POSITION));

            telemetry.addData("gripPos", gripPos);
            telemetry.addData("vertPow", vertPow);

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
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

            telemetry.update();
            dashboardTelemetry.update();
        }
    }
}
