package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Methods.DrivingMethods;

@Config
@TeleOp

public class MainTeleOp extends OpMode {
    boolean IMUReset;
    DrivingMethods Drive = new DrivingMethods();
    public void init() {
        // Allows the telemetry variable to send data to both DS and FTC dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //initialize functions with the hardware map
        Drive.init(hardwareMap);
    }

    public void loop() {
        fieldCentricDrive();

    }

    public void fieldCentricDrive() {
        IMUReset = gamepad1.options;
        double lx = gamepad1.left_stick_x * 1.1;
        double ly = gamepad1.left_stick_y;
        double rx = -gamepad1.right_stick_x;
        Drive.FieldCentric(lx, ly, rx, gamepad1.right_trigger, IMUReset, telemetry, getRuntime());
    }

    public void robotCentricDrive() {
        double lx = gamepad1.left_stick_x * 1.1;
        double ly = gamepad1.left_stick_y;
        double rx = -gamepad1.right_stick_x;

        Drive.RobotCentric(lx, ly, rx, gamepad1.right_trigger);
    }
}
