package org.firstinspires.ftc.teamcode.Methods;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;

public class LEDMethods {
    Servo led;
    public void init(HardwareMap hardwareMap) {
        led = hardwareMap.get(Servo.class, "LED");
    }

    public void color(double color) {
        led.setPosition(color);
    }

    public void redToGreen(double val) {
        led.setPosition(0.277+(val*0.223));
    }
}
