package org.firstinspires.ftc.teamcode.Methods;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ThreeDeadWheelLocalizer;

@Config
public class LEDMethods {
    Servo led0;
    Servo led1;

    public static double lowestColor = 0.277;
    public static double highestColor = 0.5;

    public static double purpleBallColor = 0.722;
    public static double greenBallColor = 0.5;

    public void init(HardwareMap hardwareMap) {
        led0 = hardwareMap.get(Servo.class, "LED0");
        led1 = hardwareMap.get(Servo.class, "LED1");
    }

    public void color(double color) {
        led0.setPosition(color);
    }

    public void redToGreen(double val) {
        //led.setPosition(Math.random());
        if (val == 1) {
            led0.setPosition(0.5);
        } else if(val == 0.5) {
            led0.setPosition(0.278);
        } else {
            led0.setPosition(0);
        }

    }

    public void ballColor(double val) {
        if (val == 1) {
            led1.setPosition(purpleBallColor);
        } else if (val == 0.5) {
            led1.setPosition(greenBallColor);
        } else {
            led1.setPosition(0.0); // 0.0 = LED is Off.
        }
    }
}
