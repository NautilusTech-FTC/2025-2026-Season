package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Methods.DrivingMethods;
import org.firstinspires.ftc.teamcode.Methods.IntakeMethods;
import org.firstinspires.ftc.teamcode.Methods.LEDMethods;
import org.firstinspires.ftc.teamcode.Methods.ShooterMethods;
import org.firstinspires.ftc.teamcode.Methods.TransferMethods;
import org.firstinspires.ftc.teamcode.Methods.VisionMethods;

import java.util.List;

@Config
@TeleOp

public class MainTeleOp extends OpMode {

    DrivingMethods Drive = new DrivingMethods();
    IntakeMethods Intake = new IntakeMethods();
    ShooterMethods Shooter = new ShooterMethods();
    TransferMethods Transfer = new TransferMethods();
    LEDMethods LED = new LEDMethods();
    VisionMethods Vision = new VisionMethods();

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
    double runtime;
    double spoontime;
    double shooterSpeed;


    boolean driverReady = false;
    int teamColor = 0;
    boolean ctrlTeamSelectLeft;
    boolean ctrlTeamSelectRight;
    boolean ctrlTeamSelectConfirm;
    boolean ctrlAutoAimToggle;
    boolean autoAimToggle;
    boolean autoAim = false;
    double correctionValue;


    //Config variables:
    //These are static so that they can be configured in the driver station app
    public static double strafeFix = 1.1;
    public static double shortShooterPower = 1550;
    public static double longShooterPower = 1550;

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

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        gamepad1.rumble(1000);

    }
    public void start() {
        resetRuntime();
    }

    public void loop() {
        performanceTracking();
        senseVars();
        shoot();
        intake_Transfer();
        robotCentricDrive();

        telemetry.addData("Distance: ", Transfer.distance);
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
        //INFO FOR OTHER PROGRAMMERS: to optimise reads, we are repurposing this function to do BULK READS. Please try not to read sensors anywhere else in the code.
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
        ctrlTeamSelectLeft = gamepad2.dpad_left;
        ctrlTeamSelectRight = gamepad2.dpad_right;
        ctrlTeamSelectConfirm = gamepad2.dpad_up;
        ctrlAutoAimToggle = gamepad1.a;
        runtime = getRuntime();
        Shooter.position = Shooter.shooterMotor.getCurrentPosition();
        Shooter.velocity = Shooter.shooterMotor.getVelocity();
        Transfer.distance = Transfer.distanceSensor.getDistance(DistanceUnit.CM);
    }

    public void fieldCentricDrive() {
        Drive.FieldCentric(ctrlLX, ctrlLY, ctrlRX, 1-ctrlRTrig, ctrlHome, strafeFix, telemetry);
    }

    public void robotCentricDrive() {
        if (!driverReady) {
            telemetry.addLine("Please select a team:");
            telemetry.addLine("Blue <- -> Red");
            if (ctrlTeamSelectLeft) {
                teamColor = 0;
            } else if (ctrlTeamSelectRight) {
                teamColor = 1;
            }
            if (teamColor == 1) {telemetry.addLine("Current team: Red");}
            else {telemetry.addLine("Current team: Blue");}
            telemetry.addLine("Press start to confirm");
            if (ctrlTeamSelectConfirm) {
                driverReady = true;
            }
        }

        if (ctrlAutoAimToggle & autoAimToggle) {
            autoAimToggle = false;
            autoAim = !autoAim;
        } else if (!ctrlAutoAimToggle) {
            autoAimToggle = true;
        }
        correctionValue = Vision.aim(teamColor, telemetry);

        if (autoAim & !(correctionValue == 2)) {
            Drive.RobotCentric(ctrlLX * strafeFix, ctrlLY, -correctionValue, 1 - ctrlRTrig);
            if (correctionValue == 0) {
                gamepad1.rumble(50);
            }
        } else {
            Drive.RobotCentric(ctrlLX * strafeFix, ctrlLY, -ctrlRX, 1-ctrlRTrig);
        }

        gamepad1.rumble(50);
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
    }

    public void shoot() {
        spoontime = runtime-spoonRunTime;
        if (ctrlSpoon & (spoonPhase == 0)) {
            spoonRunTime = runtime;
            spoonPhase = 1;
            Transfer.spoonPos(0.86); //spoon up
        }

        if ((spoontime > 0.1) & (spoonPhase == 1)) {
            Transfer.spoonPos(1); //spoon down
            spoonPhase++;
        }

        if (spoontime > 0.4) {
            spoonPhase = 0;
        }

        if (ctrlStartShootMotorS) {
            /*Shooter.motorPower(Shooter.PID(targetSpeed,Shooter.getPos(),getRuntime(),new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()))); // start spinning shooter if it's not already spinning
            //TODO: works best for long, make long shooter power this
            targetSpeed = 150; */

            Shooter.motorVelocity(shortShooterPower);
        } //The "desired speed" for PID is actually the target speed. Otherwise, the motor speed would go negative, so we can't use "ShortShootPower" and "LongShoot

        if (ctrlStartShootMotorL) {
            /*Shooter.motorPower(longShooterPower); // start spinning shooter if it's not already spinning
            //TODO: way too powerful
            targetSpeed = 150;*/

            Shooter.motorVelocity(longShooterPower);
        }

        if (ctrlStopShootMotor) {
            Shooter.motorPower(0.0);
        }

        shooterSpeed = Shooter.getSpeed(runtime);
        if (shooterSpeed >= 1480 && shooterSpeed <= 1560) {
            LED.redToGreen(1); // Makes light blue only if shooter is between the sweet spot speed range.
        } else {
            LED.redToGreen(0.1);
        }

        telemetry.addData("Shooter Speed: ",shooterSpeed);
    }
}
