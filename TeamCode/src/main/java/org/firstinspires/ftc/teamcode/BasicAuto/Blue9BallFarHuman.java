package org.firstinspires.ftc.teamcode.BasicAuto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name="BLUE 9 Ball Far HUMAN PLAYER", group="9 Ball Autos")
@Config
public class Blue9BallFarHuman extends LinearOpMode {
    public static double shootAngle = 3.575;
    public static double x1 = 41;
    public static double y1 = -57;
    public static double arriveY = -53.5;
    public static double arriveTan = -1.35;
    public static double flickTan = -1.74;




    public void runOpMode () {
        Pose2d initialPose = new Pose2d(62, -12, Math.PI);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        AutoMethods.Shoot shoot = new AutoMethods.Shoot(hardwareMap);
        AutoMethods.Intake intake = new AutoMethods.Intake(hardwareMap);
        AutoMethods.Combined combined = new AutoMethods.Combined(hardwareMap);

        TrajectoryActionBuilder aim = drive.actionBuilder(initialPose)
                .setTangent(Math.PI)
                .splineToLinearHeading(new Pose2d(55, -12, shootAngle), shootAngle, new TranslationalVelConstraint(40), new ProfileAccelConstraint(-50, 50));

        TrajectoryActionBuilder hp3 = aim.endTrajectory().fresh()
                .setTangent(0)
                .splineToSplineHeading(new Pose2d(68, -25, -Math.PI/2), -Math.PI/2, new TranslationalVelConstraint(75.0), new ProfileAccelConstraint(-75, 75))
                .setTangent(-Math.PI/2)
                .splineTo(new Vector2d(69, arriveY), arriveTan, new TranslationalVelConstraint(75.0), new ProfileAccelConstraint(-75, 75))
                .turnTo(flickTan)
                .splineTo(new Vector2d(68, -58), -Math.PI/2)
                .setTangent(Math.PI/2)
                .splineToLinearHeading(new Pose2d(55, -12, shootAngle), Math.PI/2, new TranslationalVelConstraint(50));

        TrajectoryActionBuilder row1 = aim.endTrajectory().fresh()
                .setTangent(shootAngle)
                .splineToSplineHeading(new Pose2d(x1, -34, -Math.PI/2), -Math.PI/2, new TranslationalVelConstraint(75.0), new ProfileAccelConstraint(-75, 75))
                .lineToY(y1, new TranslationalVelConstraint(25.0))
                .setTangent(Math.PI/2)
                .splineToLinearHeading(new Pose2d(55, -12, shootAngle), shootAngle-Math.PI, new TranslationalVelConstraint(40), new ProfileAccelConstraint(-50, 50));

        TrajectoryActionBuilder home = aim.endTrajectory().fresh()
                .setTangent(-Math.PI/2)
                .splineToLinearHeading(new Pose2d(60, -35, Math.PI), -Math.PI/2, new TranslationalVelConstraint(75.0), new ProfileAccelConstraint(-75, 75));




        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        //Shoot first three
                        new ParallelAction(
                                shoot.shooterOn(1575),
                                shoot.holySpoonDown(),
                                aim.build()
                        ),
                        new SleepAction(1),
                        combined.shoot1(),
                        combined.shoot1(),
                        combined.shoot1(),
                        //Pickup row 1 & shoot
                        intake.spinIn(),
                        intake.transSpinIn(),
                        hp3.build(),
                        intake.spinStop(),
                        intake.transSpinStop(),
                        combined.shoot1(),
                        combined.shoot1(),
                        combined.shoot1(),
                        //Pickup row 2 & shoot
                        intake.transSpinIn(),
                        intake.spinIn(),
                        row1.build(),
                        intake.transSpinStop(),
                        intake.spinStop(),
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
