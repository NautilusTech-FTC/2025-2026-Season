package org.firstinspires.ftc.teamcode.BasicAuto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

public class ShootPreloadBlueFar extends LinearOpMode {
    public void runOpMode () {
        Pose2d initialPose = new Pose2d(62, -15, Math.PI);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        int shootPosX = 55;
        double shootAngle = 3.93;

        TrajectoryActionBuilder aim = drive.actionBuilder(initialPose)
                .lineToX(shootPosX)
                .turnTo(shootAngle);
    }
}
