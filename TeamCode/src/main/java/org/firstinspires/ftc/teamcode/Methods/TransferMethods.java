package org.firstinspires.ftc.teamcode.Methods;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class TransferMethods {

    CRServo transferServo;
    Servo spoonServo;

    public DistanceSensor distanceSensor;
    public double distance;

    public void init (HardwareMap hardwareMap) {
        transferServo = hardwareMap.get(CRServo.class, "TransferServo");
        spoonServo = hardwareMap.get(Servo.class, "SpoonServo");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "DistanceSensor");
    }

    public void servoPower (double power) {
        transferServo.setPower(power);
    }

    public void spoonPos (double position) {
        spoonServo.setPosition(position);
    }


}
