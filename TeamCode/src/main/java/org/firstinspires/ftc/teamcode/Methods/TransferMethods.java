package org.firstinspires.ftc.teamcode.Methods;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class TransferMethods {

    CRServo transferServo;

    //TODO: change to what we actually want, this is a test value
    double servoSpeed = 0.75;

    public void init (HardwareMap hwMap) {
        transferServo = hwMap.get(CRServo.class, "TransferServo");
    }

    public void spin () {
        transferServo.setPower(servoSpeed);
    }

    public void stopSpinning () {
        transferServo.setPower(0);
    }
}
