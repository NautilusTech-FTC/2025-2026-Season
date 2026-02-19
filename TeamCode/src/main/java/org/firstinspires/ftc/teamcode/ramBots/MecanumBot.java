package org.firstinspires.ftc.teamcode.ramBots;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Methods.DrivingMethods;


@TeleOp
public class MecanumBot extends OpMode {
    DrivingMethods Drive = new DrivingMethods();

    double strafeFix = 1.1;

    double ctrlLX;
    double ctrlLY;
    double ctrlRX;
    double ctrlRT;
    boolean ctrlOptions;

    public void init() {
        Drive.init(hardwareMap);
    }

    public void loop() {
        ctrlLX = gamepad1.left_stick_x;
        ctrlLY = gamepad1.left_stick_y;
        ctrlRX = gamepad1.right_stick_x;
        ctrlRT = gamepad1.right_trigger;
        ctrlOptions = gamepad1.options;
        FieldCentric();
    }

    public void FieldCentric() {
        Drive.RobotCentric(ctrlLX * strafeFix, ctrlLY, -ctrlRX, 0.8);
    }
}
