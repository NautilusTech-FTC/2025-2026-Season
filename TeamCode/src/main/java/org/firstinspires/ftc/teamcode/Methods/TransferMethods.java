package org.firstinspires.ftc.teamcode.Methods;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class TransferMethods {

    CRServo transferServo;
    Servo spoonServo;

    public void init (HardwareMap hardwareMap) {
        transferServo = hardwareMap.get(CRServo.class, "TransferServo");
        spoonServo = hardwareMap.get(Servo.class, "SpoonServo");
    }

    public void servoPower (double power) {
        transferServo.setPower(power);
    }

    public void spoonPos (double position) {
        spoonServo.setPosition(position);
    }
}
