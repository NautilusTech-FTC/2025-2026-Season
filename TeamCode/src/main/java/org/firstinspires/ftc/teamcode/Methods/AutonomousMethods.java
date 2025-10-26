package org.firstinspires.ftc.teamcode.Methods;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class AutonomousMethods {

    private DcMotor fL;
    private DcMotor fR;
    private DcMotor bR;
    private DcMotor bL;

    public void init(HardwareMap hardwareMap) {

        fL = hardwareMap.get(DcMotor.class, "Motor0");
        fR = hardwareMap.get(DcMotor.class, "Motor1");
        bR = hardwareMap.get(DcMotor.class, "Motor2");
        bL = hardwareMap.get(DcMotor.class, "Motor3");

        RevHubOrientationOnRobot RevOrientation =
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,RevHubOrientationOnRobot.UsbFacingDirection.RIGHT);
    }

    public void setDriveAngle() {
    //Didn't make full speed so it doesn't break wall, in case it crashes.
        fL.setPower(0.5);
        fR.setPower(0.5);
        bR.setPower(0.5);
        bL.setPower(0.5);

    }

    public void stopMotors() {
        fL.setPower(0);
        fR.setPower(0);
        bR.setPower(0);
        bL.setPower(0);

        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
}
