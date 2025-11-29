package org.firstinspires.ftc.teamcode.Methods;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ShooterMethods {
    public DcMotorEx shooterMotor;

    private double oldPos;
    public double position;
    private double speed;
    private double lastCheck;

    double integSum = 0; // Related to kI.
    double kP = 0.1; // Proportional Term - Difference between our current speed and our target speed.
    double kI = 0; // Integral Term - What our constant error value is, gets us closer to target over time.
    double kD = 0; // Derivative Term - Predicts how much we need to increase/decrease to hit desired value, prevents overshooting.
    double preTime;
    private double lastError = 0;

    private double currentSpeed;


    public void init(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "ShooterMotor");
        shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooterMotor.setDirection(DcMotorEx.Direction.FORWARD);


    }

    public void motorPower(double power) {
        shooterMotor.setPower(power);
    }

    public void motorVelocity(double vel) {
        shooterMotor.setVelocity(vel);
    }

    public void fetch() {

    }

    public double getSpeed(double runtime) {
        if((runtime-lastCheck)>0.1) {
            lastCheck = runtime;
            speed = position - oldPos;
            oldPos = position;
        }
        return (speed);
    }

    public double getPos() {
        return (position);
    }






}