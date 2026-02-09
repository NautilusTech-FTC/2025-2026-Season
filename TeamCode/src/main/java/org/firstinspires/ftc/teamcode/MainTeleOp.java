package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.Methods.DrivingMethods;
import org.firstinspires.ftc.teamcode.Methods.IntakeMethods;
import org.firstinspires.ftc.teamcode.Methods.LEDMethods;
import org.firstinspires.ftc.teamcode.Methods.ShooterMethods;
import org.firstinspires.ftc.teamcode.Methods.TiltMethods;
import org.firstinspires.ftc.teamcode.Methods.TransferMethods;
import org.firstinspires.ftc.teamcode.Methods.VisionMethods;

import java.util.List;

@Config
@TeleOp(name="MainTeleOp", group="Main")
public class MainTeleOp extends OpMode {

    //Methods:
    DrivingMethods Drive = new DrivingMethods();
    IntakeMethods Intake = new IntakeMethods();
    ShooterMethods Shooter = new ShooterMethods();
    TransferMethods Transfer = new TransferMethods();
    TransferMethods.DetectedColor detectedColor;
    LEDMethods LED = new LEDMethods();
    VisionMethods Vision = new VisionMethods();
    TiltMethods Tilt = new TiltMethods();

    //Control variables:
    //These contain controller inputs
    double ctrlLX, ctrlLY, ctrlRX;
    double ctrlRTrig;
    boolean ctrlHome;
    boolean ctrlTransSpinner;
    boolean ctrlUnTransSpinner;
    boolean ctrlSpoon;
    boolean ctrlStopShootMotor;
    boolean ctrlStartShootMotorS;
    boolean ctrlStartShootMotorL;
    boolean ctrlTilt;

    //Shooter timing variables:
    double spoonRunTime;
    int spoonPhase;
    double runtime;
    double spoontime;
    double shooterSpeed;
    boolean shooterEnable;

    //Misc.:
    int teamColor = 0;
    boolean ctrlTeamSelectLeft = false;
    boolean ctrlTeamSelectRight = false;
    boolean ctrlAutoAimToggle;
    boolean autoAimToggle;
    boolean autoAim = false;
    double correctionValue;
    double lightVal = 0;
    boolean tiltOn = false;

    // PIDF:
    public double highShooterVelocity = 1620;
    double curveTargetVelocity = highShooterVelocity;
    public double P = 293;
    public double I = 0;
    public double D = 0.5;
    public double F = 15.57;
    double error;

    //Config variables:
    //These are static so that they can be configured
    public static double strafeFix = 1.1;

    //Variables for performance tracking:
    int performanceCycles = 0;
    double lastTime;
    double peakCycle = 0;

    public void init() {
        // Allows the telemetry variable to send data to both DS and FTC dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //initialize functions with the hardware map
        Drive.init(hardwareMap);
        Intake.init(hardwareMap);
        Transfer.init(hardwareMap);
        Shooter.init(hardwareMap);
        LED.init(hardwareMap);
        Vision.init(hardwareMap);
        Tilt.init(hardwareMap);

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

    }
    public void start() {
        resetRuntime();
        Transfer.spoonPos(0.97);
    }

    public void loop() {
        performanceTracking();
        senseVars();
        shoot();
        intake_Transfer();
        robotCentricDrive();
        tilt();
    }

    public void performanceTracking() {
        performanceCycles++;
        if (runtime-lastTime>peakCycle) {
            peakCycle = runtime-lastTime;
        }
        telemetry.addData("last cycle", runtime-lastTime);
        telemetry.addData("average cycle", runtime/performanceCycles);
        telemetry.addData("peak cycle", peakCycle);
        lastTime = runtime;
    }

    public void senseVars() {
        //INFO FOR OTHER PROGRAMMERS: to optimise reads, we are using this function to do BULK READS. Please try not to read sensors anywhere else in the code.
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
        ctrlTeamSelectLeft = gamepad1.dpad_left;
        ctrlTeamSelectRight = gamepad1.dpad_right;
        ctrlAutoAimToggle = gamepad1.a;
        ctrlTilt = gamepad1.y;
        runtime = getRuntime();
        Shooter.position = Shooter.shooterMotor.getCurrentPosition();
        Shooter.velocity = Shooter.shooterMotor.getVelocity();
        Tilt.tiltPos = Tilt.tiltMotor.getCurrentPosition();
    }

    /*
    public void fieldCentricDrive() {
        Drive.FieldCentric(ctrlLX, ctrlLY, ctrlRX, 1-ctrlRTrig, ctrlHome, strafeFix, telemetry);
    }
    */

    public void robotCentricDrive() {
        
        if (ctrlTeamSelectLeft) {
            teamColor = 0;
        } else if (ctrlTeamSelectRight) {
            teamColor = 1;
        }
        if (teamColor == 1) {telemetry.addLine("Current team: Red");}
        else {telemetry.addLine("Current team: Blue");}

        if (ctrlAutoAimToggle & autoAimToggle) {
            autoAimToggle = false;
            autoAim = !autoAim;
        } else if (!ctrlAutoAimToggle) {
            autoAimToggle = true;
        }
        correctionValue = Vision.aim(teamColor, telemetry);

        if (autoAim & !(correctionValue == 2)) {
            Drive.RobotCentric(ctrlLX * strafeFix, ctrlLY, correctionValue, 1 - ctrlRTrig);
        } else {
            Drive.RobotCentric(ctrlLX * strafeFix, ctrlLY, -ctrlRX, 1-ctrlRTrig);
        }

        if (autoAim) {
            LED.redToGreen(1);
        } else {
            LED.redToGreen(0);
        }
    }

    public void intake_Transfer() {
        if (ctrlTransSpinner) {
            Intake.motorPower(-1.0);
            Transfer.servoPower(-1.0);
        } else if (ctrlUnTransSpinner) {
            Intake.motorPower(1.0);
            Transfer.servoPower(1.0);
        } else {
            if (spoonPhase == 2) { //This is to ensure that the ball does stuff after the spoon shoots
                Transfer.servoPower(-1.0);
                Intake.motorPower(-1.0);
            } else {
            Intake.motorPower(0.0);
            Transfer.servoPower(0.0);
            }
        }

        detectedColor = Transfer.getDetectedColor(telemetry);

        telemetry.addData("Color: ", detectedColor);

        if (detectedColor == TransferMethods.DetectedColor.PURPLE) {
            lightVal = 1;
        } else if (detectedColor == TransferMethods.DetectedColor.GREEN) {
            lightVal = 0.5;
        } else {
            lightVal = 0;
        }

        LED.ballColor(lightVal);
    }

    public void shoot() {
        shooterSpeed = Shooter.velocity;

        spoontime = runtime-spoonRunTime;
        if ((ctrlSpoon & (spoonPhase == 0)) & ((shooterSpeed > shooterSpeed - 50) & (shooterSpeed < shooterSpeed + 25))) {
            spoonRunTime = runtime;
            spoonPhase = 1;
            Transfer.spoonPos(0.84); //spoon up
        }

        if ((spoontime > 0.15) & (spoonPhase == 1)) {
            Transfer.spoonPos(0.97); //spoon down
            spoonPhase++;
        }

        if (spoontime > 0.3) {
            spoonPhase = 0;
        }

        error = curveTargetVelocity - shooterSpeed;
        if (ctrlStartShootMotorL || ctrlStartShootMotorS) {
            Shooter.motorVelocity(curveTargetVelocity);
            shooterEnable = true;
        }

        if (ctrlStopShootMotor) {
            Shooter.motorPower(0.0);
            shooterEnable = false;
        }

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,I,D,F);
        Shooter.shooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        telemetry.addData("Shooter Speed: ",shooterSpeed);

        // PIDF Telemetry:
        telemetry.addData("Target Speed: ", curveTargetVelocity);
        telemetry.addData("Error: ", "%.2f", error);
    }

    public void tilt() {
        if (ctrlTilt && !tiltOn) {
            Tilt.liftPos(100);
            tiltOn = true;
        }
        if (ctrlTilt && tiltOn) {
            Tilt.liftPos(0);
            tiltOn = false;
        }

        telemetry.addData("Tilt Pos: ", Tilt.tiltPos);
    }
}
