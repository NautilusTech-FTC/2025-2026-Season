package org.firstinspires.ftc.teamcode.ramBots;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Methods.DrivingMethods;


@TeleOp
public class MecanumBot extends OpMode {
    DrivingMethods Drive = new DrivingMethods();
    double ctrlLX = gamepad1.left_stick_x;
    double ctrlLY = gamepad1.left_stick_y;
    double ctrlRX = gamepad1.right_stick_x;
    double ctrlRT = gamepad1.right_trigger;
    boolean ctrlOptions = gamepad1.options;
    double strafeFix = 1.1;

    public void init() {
        Drive.init(hardwareMap);
    }

    public void loop() {
        FieldCentric();
    }

    public void FieldCentric() {
        Drive.FieldCentric(ctrlLX,ctrlLY,ctrlRX,1-ctrlRT,ctrlOptions,strafeFix,telemetry);
    }
}
