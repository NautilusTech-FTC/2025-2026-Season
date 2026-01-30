package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Disabled
public class zeroServo extends OpMode {
    Servo spoonServo;
    public void init() {
        spoonServo = hardwareMap.get(Servo.class, "SpoonServo");
        spoonServo.setPosition(0);
    }

    public void loop() {

    }
}
