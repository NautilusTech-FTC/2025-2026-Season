package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class oneServo extends OpMode {
    Servo spoonServo;
    public void init() {
        spoonServo = hardwareMap.get(Servo.class, "SpoonServo");
        spoonServo.setPosition(1);
    }

    public void loop() {

    }
}
