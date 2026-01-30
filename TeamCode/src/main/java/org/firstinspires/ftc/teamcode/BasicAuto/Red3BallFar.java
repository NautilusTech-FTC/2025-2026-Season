package org.firstinspires.ftc.teamcode.BasicAuto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;


@Autonomous(name="RED 3 Ball Far", group="3 Ball Autos")
@Config
public class Red3BallFar extends LinearOpMode {
    public static double shootAngle = 2.75;
    public static double x1 = 41;
    public static double x2 = 15;
    public static double y1 = 64;
    public static double y2 = 64;
    public static double shootSpeed = 1550;

    public void runOpMode () {
        Pose2d initialPose = new Pose2d(62, 15, Math.PI);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        AutoMethods.Shoot shoot = new AutoMethods.Shoot(hardwareMap);
        AutoMethods.Intake intake = new AutoMethods.Intake(hardwareMap);
        AutoMethods.Combined combined = new AutoMethods.Combined(hardwareMap);

        TrajectoryActionBuilder aim = drive.actionBuilder(initialPose)
                .setTangent(Math.PI)
                .splineToLinearHeading(new Pose2d(55, 15, shootAngle), shootAngle);

        TrajectoryActionBuilder home = aim.endTrajectory().fresh()
                .setTangent(-Math.PI/2)
                .splineToLinearHeading(new Pose2d(60, 35, Math.PI), -Math.PI/2);


        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        //Shoot first three
                        new ParallelAction(
                                shoot.shooterOn(shootSpeed),
                                shoot.holySpoonDown(),
                                aim.build()
                        ),
                        new SleepAction(1.5),
                        combined.shoot1(),
                        combined.shoot1(),
                        combined.shoot1(),
                        //End auto
                        shoot.shooterOff(),
                        home.build()
                )
        );
    }
}
