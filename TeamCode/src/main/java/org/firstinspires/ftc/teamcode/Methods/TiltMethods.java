package org.firstinspires.ftc.teamcode.Methods;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TiltMethods {
    public DcMotorEx tiltMotor;
    public double tiltPos;

    public void init(HardwareMap hardwareMap) {
        tiltMotor = hardwareMap.get(DcMotorEx.class, "TiltMotor");
        tiltMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        tiltMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        tiltMotor.setTargetPosition(0);
        tiltMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void liftToPos(int pos) {
        tiltMotor.setTargetPosition(pos);
        tiltMotor.setPower(1);
    }
}
