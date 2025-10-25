package org.firstinspires.ftc.teamcode.Methods;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class AutonomousMethods {

    private DcMotor motor0;
    private DcMotor motor1;
    private DcMotor motor2;
    private DcMotor motor3;

    public void init(HardwareMap hardwareMap) {

        motor0 = hardwareMap.get(DcMotor.class, "Motor0");
        motor1 = hardwareMap.get(DcMotor.class, "Motor1");
        motor2 = hardwareMap.get(DcMotor.class, "Motor2");
        motor3 = hardwareMap.get(DcMotor.class, "Motor3");

        RevHubOrientationOnRobot RevOrientation =
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,RevHubOrientationOnRobot.UsbFacingDirection.RIGHT);
    }

    public void setDriveAngle() {
    //Didn't make full speed so it doesn't break wall, in case it crashes.
        motor0.setPower(0.5);
        motor1.setPower(0.5);
        motor2.setPower(0.5);
        motor3.setPower(0.5);

    }

    public void stopMotors() {
        motor0.setPower(0);
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);

        motor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
}
