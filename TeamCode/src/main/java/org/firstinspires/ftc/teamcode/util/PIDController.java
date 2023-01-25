package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {

    private double Kp, Ki, Kd;
    private double integralSum, lastError;

    ElapsedTime timer = new ElapsedTime();

    /**
     * construct PID Controller
     * @param Kp Proportional coefficient
     * @param Ki Integral coefficient
     * @param Kd Derivative coefficient
     */
    public PIDController(double Kp, double Ki, double Kd) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
    }

    /**
     * Update the PID Controller Output
     * @param target where you want to be - the reference
     * @param state where you are - current position (encoder)
     * @return the command to the motor - motor power
     */
    public double update(double target, double state) {

        // Calculate error
        double error = target - state;

        // Kd value
        double derivative = (error - lastError) / timer.seconds();

        // Ki value
        integralSum = Ki + (error * timer.seconds());

        double out = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

        return(out);
    }
}
