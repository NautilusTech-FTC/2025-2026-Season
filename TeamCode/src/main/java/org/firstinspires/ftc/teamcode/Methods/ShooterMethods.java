package org.firstinspires.ftc.teamcode.Methods;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ShooterMethods {
    DcMotor shooterMotor;

    public void init(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(DcMotor.class, "ShooterMotor");
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void motorPower(double power) {
        shooterMotor.setPower(power);
    }
}