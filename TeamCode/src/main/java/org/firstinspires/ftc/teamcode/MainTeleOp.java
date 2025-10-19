package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Methods.DrivingMethods;
import org.firstinspires.ftc.teamcode.Methods.IntakeMethods;

@Config
@TeleOp

public class MainTeleOp extends OpMode {

    DrivingMethods Drive = new DrivingMethods();
    IntakeMethods Intake = new IntakeMethods();

    double ctrlLX;
    double ctrlLY;
    double ctrlRX;
    double ctrlRTrig;
    boolean ctrlHome;
    boolean ctrlIntake;


    //Config variables:
    double strafeFix = 1.1;

    public void init() {
        // Allows the telemetry variable to send data to both DS and FTC dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //initialize functions with the hardware map
        Drive.init(hardwareMap);
        Intake.init(hardwareMap);
    }

    public void loop() {
        controlVars();
        fieldCentricDrive();
        intake();
        //robotCentricDrive();

    }

    public void controlVars() {
        ctrlLX = gamepad1.left_stick_x; //Robot move Y
        ctrlLY = gamepad1.left_stick_y; //Robot move Y
        ctrlRX = gamepad1.right_stick_x; //Robot rotation
        ctrlRTrig = gamepad1.right_trigger; //Robot speed
        ctrlHome = gamepad1.options; //IMU reset for field centric
        ctrlIntake = gamepad1.a; //Intake
    }

    public void intake() {
        if (ctrlIntake) {
            Intake.motorPower(1.0);
            telemetry.addData("intake", true);
        } else {
            Intake.motorPower(0.0);
        }

    }

    public void fieldCentricDrive() {
        Drive.FieldCentric(ctrlLX, ctrlLY, -ctrlRX, 1-ctrlRTrig, ctrlHome, strafeFix, telemetry);
    }

    public void robotCentricDrive() {
        Drive.RobotCentric(ctrlLX * strafeFix, ctrlLY, -ctrlRX, 1-ctrlRTrig);
    }
}
