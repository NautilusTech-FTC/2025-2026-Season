package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Methods.DrivingMethods;
import org.firstinspires.ftc.teamcode.Methods.IntakeMethods;
import org.firstinspires.ftc.teamcode.Methods.LEDMethods;
import org.firstinspires.ftc.teamcode.Methods.ShooterMethods;
import org.firstinspires.ftc.teamcode.Methods.TransferMethods;

@Config
@TeleOp

public class MainTeleOp extends OpMode {

    DrivingMethods Drive = new DrivingMethods();
    IntakeMethods Intake = new IntakeMethods();
    ShooterMethods Shooter = new ShooterMethods();
    TransferMethods Transfer = new TransferMethods();
    LEDMethods LED = new LEDMethods();

    double ctrlLX;
    double ctrlLY;
    double ctrlRX;
    double ctrlRTrig;
    boolean ctrlHome;
    boolean ctrlTransSpinner;
    boolean ctrlUnTransSpinner;
    boolean ctrlSpoon;
    boolean ctrlStopShootMotor;
    boolean ctrlStartShootMotorS;
    boolean ctrlStartShootMotorL;
    double spoonRunTime;
    int spoonPhase;
    double targetSpeed;

    double shooterSpeed;


    //Config variables:
    //These are static so that they can be configured in the driver station app
    static double strafeFix = 1.1;
    static double shortShooterPower = 0.75;
    static double longShooterPower = 0.85;


    public void init() {
        // Allows the telemetry variable to send data to both DS and FTC dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //initialize functions with the hardware map
        Drive.init(hardwareMap);
        Intake.init(hardwareMap);
        Transfer.init(hardwareMap);
        Shooter.init(hardwareMap);
        LED.init(hardwareMap);
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
        ctrlLX = gamepad1.left_stick_x; //Robot move X
        ctrlLY = gamepad1.left_stick_y; //Robot move Y
        ctrlRX = gamepad1.right_stick_x; //Robot rotation
        ctrlRTrig = gamepad1.right_trigger; //Robot speed
        ctrlHome = gamepad1.options; //IMU reset for field centric
        ctrlTransSpinner = gamepad2.left_bumper; //Intake and flywheel
        ctrlUnTransSpinner = gamepad2.right_bumper; //Eject intake
        ctrlSpoon = gamepad2.y; //Spin shooter and move holy spoon (shoots)
        ctrlStopShootMotor = gamepad2.b; //Stop shooter spinning
        ctrlStartShootMotorS = gamepad2.x; // Short range motor
        ctrlStartShootMotorL = gamepad2.a; // Long range motor
    }

    public void fieldCentricDrive() {
        Drive.FieldCentric(ctrlLX, ctrlLY, ctrlRX, 1-ctrlRTrig, ctrlHome, strafeFix, telemetry);
    }

    public void robotCentricDrive() {
        Drive.RobotCentric(ctrlLX * strafeFix, ctrlLY, -ctrlRX, 1-ctrlRTrig);
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
            if (spoonPhase == 2) { //This is to ensure that the ball does stuff after the spoon shoots
                Transfer.servoPower(-1.0);
            } else {
            Intake.motorPower(0.0);
            Transfer.servoPower(0.0);
            }
        }
    }

    public void shoot() {
        if (ctrlSpoon & (spoonPhase == 0)) {
            spoonRunTime = getRuntime();
            spoonPhase = 1;
            Transfer.spoonPos(0.8); //spoon up
        }


        if ((getRuntime()-spoonRunTime > 0.5) & (spoonPhase == 1)) {
            Transfer.spoonPos(1); //spoon down
            spoonPhase++;
        }

        if (getRuntime()-spoonRunTime > 1.5) {
            spoonPhase = 0;
        }


        if (ctrlStartShootMotorS) {
            Shooter.motorPower(Shooter.PID(shortShooterPower,Shooter.getPos(),getRuntime(),new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()))); // start spinning shooter if it's not already spinning
            //TODO: works best for long, make long shooter power this
            targetSpeed = 150;
        }

        if (ctrlStartShootMotorL) {
            Shooter.motorPower(longShooterPower); // start spinning shooter if it's not already spinning
            //TODO: way too powerful
            targetSpeed = 165;
        }

        if (ctrlStopShootMotor) {
            Shooter.motorPower(0.0);
        }

        shooterSpeed = Shooter.getSpeed(getRuntime());
        telemetry.addData("Shooter Speed:", shooterSpeed);
        if (shooterSpeed >= targetSpeed) {
            LED.redToGreen(1);
        } else {
            LED.redToGreen(0.1);
        }
    }
}
