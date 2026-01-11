package org.firstinspires.ftc.teamcode.BasicAuto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name="RED Close", group="Drive autos")
public class RedClose extends LinearOpMode {
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(-51, 49.5, Math.toRadians(307));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder action = drive.actionBuilder(initialPose)
                .lineToX(-48)
                .turnTo(0)
                .lineToX(-30);


        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        action.build()
                )
        );
    }
}
