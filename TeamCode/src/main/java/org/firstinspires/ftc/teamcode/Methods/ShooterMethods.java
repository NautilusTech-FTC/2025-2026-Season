package org.firstinspires.ftc.teamcode.Methods;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ShooterMethods {
    DcMotorEx shooterMotor;

    private double pos;
    private double speed;
    private double lastCheck;

    double integSum = 0; // Related to kI.
    double kP = 0; // Proportional Term - Distance between our location and our desired location.
    double kI = 0; // Integral Term - What our constant error value is.
    double kD = 0; // Derivative Term - Predicts how much we need to increase/decrease to hit desired value.
    double preTime;
    private double lastError = 0;

    public void init(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "ShooterMotor");
        shooterMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void motorPower(double power) {
        shooterMotor.setPower(power);
    }

    public double getSpeed(double runtime) {
        if((runtime-lastCheck)>0.1) {
            lastCheck = runtime;
            speed = shooterMotor.getCurrentPosition() - pos;
            pos = shooterMotor.getCurrentPosition();
        }
        return (speed);
    }

    public double getPos() {
        return (shooterMotor.getCurrentPosition());
    }

    public double PID(double desiredPos, double currentPos, double time, Telemetry telemetry) {
        double error = desiredPos - currentPos;
        integSum += error * (time - preTime);
        double derivative = (error - lastError)/(time - preTime);

        lastError = error;
        preTime = time;

        double output = (error * kP) + (derivative * kD) + (integSum * kI);

        telemetry.addData("desiredPos",desiredPos);
        return output;

        // Set motor power to PID controller.
    }
}