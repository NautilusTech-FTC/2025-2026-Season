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

    DrivingMethods Drive = new DrivingMethods();

    double ctrlLX;
    double ctrlLY;
    double ctrlRX;
    double ctrlRTrig;
    boolean ctrlHome;

    public void init() {
        // Allows the telemetry variable to send data to both DS and FTC dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //initialize functions with the hardware map
        Drive.init(hardwareMap);
    }

    public void loop() {
        controlVars();
        fieldCentricDrive();

    }

    public void controlVars() {
        ctrlLX = gamepad1.left_stick_x * 1.1; //Robot move Y
        ctrlLY = gamepad1.left_stick_y; //Robot move Y
        ctrlRX = gamepad1.right_stick_x; //Robot rotation
        ctrlRTrig = gamepad1.right_trigger; //Robot speed
        ctrlHome = gamepad1.options; //IMU reset for field centric
    }

    public void fieldCentricDrive() {
        Drive.FieldCentric(ctrlLX * 1.1, ctrlLY, -ctrlRX, 1-ctrlRTrig, ctrlHome, telemetry);
    }

    public void robotCentricDrive() {
        Drive.RobotCentric(ctrlLX * 1.1, ctrlLY, -ctrlRX, 1-ctrlRTrig);
    }
}
