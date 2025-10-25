package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Methods.DrivingMethods;
import org.firstinspires.ftc.teamcode.Methods.IntakeMethods;
import org.firstinspires.ftc.teamcode.Methods.ShooterMethods;
import org.firstinspires.ftc.teamcode.Methods.TransferMethods;

@Config
@TeleOp

public class MainTeleOp extends OpMode {

    DrivingMethods Drive = new DrivingMethods();
    IntakeMethods Intake = new IntakeMethods();
    ShooterMethods Shooter = new ShooterMethods();
    TransferMethods Transfer = new TransferMethods();

    double ctrlLX;
    double ctrlLY;
    double ctrlRX;
    double ctrlRTrig;
    boolean ctrlHome;
    boolean ctrlTransSpinner;
    boolean ctrlUnTransSpinner;
    boolean ctrlSpoon;
    boolean stopShootMotor;
    boolean startShootMotor;
    double spoonRunTime;
    int spoonPhase;


    //Config variables:
    double strafeFix = 1.1;
    double ctrlShooterPower = 0.7;

    public void init() {
        // Allows the telemetry variable to send data to both DS and FTC dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //initialize functions with the hardware map
        Drive.init(hardwareMap);
        Intake.init(hardwareMap);
        Transfer.init(hardwareMap);
        Shooter.init(hardwareMap);
        resetRuntime();
    }

    public void loop() {
        controlVars();
        //fieldCentricDrive();
        intake_Transfer();
        shoot();
        robotCentricDrive();

    }

    public void controlVars() {
        ctrlLX = gamepad1.left_stick_x; //Robot move Y
        ctrlLY = gamepad1.left_stick_y; //Robot move Y
        ctrlRX = gamepad1.right_stick_x; //Robot rotation
        ctrlRTrig = gamepad1.right_trigger; //Robot speed
        ctrlHome = gamepad1.options; //IMU reset for field centric
        ctrlTransSpinner = gamepad2.left_bumper; //Intake and flywheel
        ctrlUnTransSpinner = gamepad2.right_bumper; //Eject intake
        ctrlSpoon = gamepad2.y; //Spin shooter and move holy spoon
        startShootMotor = gamepad2.x;
        stopShootMotor = gamepad2.b; //Stop shooter spinning
    }

    public void fieldCentricDrive() {
        Drive.FieldCentric(ctrlLX, ctrlLY, ctrlRX, 1-ctrlRTrig, ctrlHome, strafeFix, telemetry);
    }

    public void robotCentricDrive() {
        Drive.RobotCentric(-ctrlLX * strafeFix, ctrlLY, -ctrlRX, 1-ctrlRTrig);
    }

    public void intake_Transfer() {
        if (ctrlTransSpinner) {
            Intake.motorPower(-1.0);
            Transfer.servoPower(-1.0);
            telemetry.addData("intake", true);
        } else if (ctrlUnTransSpinner) {
            Intake.motorPower(1.0);
            Transfer.servoPower(1.0);
        } else {
            if (spoonPhase == 3) { //This is to ensure that the ball does stuff after the spoon shoots
                Transfer.servoPower(-1.0);
            } else {
            Intake.motorPower(0.0);
            Transfer.servoPower(0.0);
            }
        }


    }

    public void shoot() {
        if (ctrlSpoon) {
            spoonRunTime = getRuntime();
            Transfer.spoonPos(1);
            spoonPhase = 1;
        }

        if (startShootMotor) {
            Shooter.motorPower(ctrlShooterPower); // start spinning shooter if it's not already spinning
        }

        if ((getRuntime()-spoonRunTime > 0.5) & (spoonPhase == 1)) {
            Transfer.spoonPos(0.8);
            spoonPhase++;
        }

        if ((getRuntime()-spoonRunTime > 1.0) & (spoonPhase == 2)) {
            Transfer.spoonPos(1);
            spoonPhase++;
        }

        if (getRuntime()-spoonRunTime > 2.0) {
            spoonPhase = 0;
        }

        if (stopShootMotor) {
            Shooter.motorPower(0.0);
        }
    }
}
