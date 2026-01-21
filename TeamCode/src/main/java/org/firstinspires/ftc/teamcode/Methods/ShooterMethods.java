package org.firstinspires.ftc.teamcode.Methods;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ShooterMethods {
    public DcMotorEx shooterMotor;

    public double position;
    public double velocity;

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

    public double getPos() {
        return (position);
    }






}