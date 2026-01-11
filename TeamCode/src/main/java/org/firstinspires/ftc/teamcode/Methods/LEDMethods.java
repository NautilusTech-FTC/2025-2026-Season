package org.firstinspires.ftc.teamcode.Methods;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ThreeDeadWheelLocalizer;

@Config
public class LEDMethods {
    Servo led;


    public static double lowestColor = 0.28;
    public static double highestColor = 0.61;




    public void init(HardwareMap hardwareMap) {
        led = hardwareMap.get(Servo.class, "LED");
    }

    public void color(double color) {
        led.setPosition(color);
    }

    public void redToGreen(double val) {
        //led.setPosition(Math.random());
        if (val >= 1) {
            led.setPosition(highestColor);
        } else {
            led.setPosition(lowestColor);
        }

    }
}
