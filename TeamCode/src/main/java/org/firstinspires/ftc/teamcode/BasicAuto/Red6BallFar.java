package org.firstinspires.ftc.teamcode.BasicAuto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
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

@Config
@Autonomous

public class Red6BallFar extends LinearOpMode {
    public static int shootPosX = 55;
    public static double shootAngle = 2.8;
    public static double shootVelocity = 1575;


    public static class Intake {
        private DcMotor motor;
        private CRServo transServo;

        public Intake (HardwareMap hardwareMap) {
            motor = hardwareMap.get(DcMotor.class, "IntakeMotor");
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            transServo = hardwareMap.get(CRServo.class, "TransferServo");
        }
        public Action spinIn () {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    motor.setPower(-1);
                    return false;
                }
            };
        }
        public Action spinOut () {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    motor.setPower(1);
                    return false;
                }
            };
        }
        public Action spinStop () {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    motor.setPower(0);
                    return false;
                }
            };
        }
        public Action transSpinIn () {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    transServo.setPower(-1);
                    packet.addLine("TransSpinIn");
                    return false;
                }
            };
        }
        public Action transSpinOut () {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    transServo.setPower(1);
                    return false;
                }
            };
        }
        public Action transSpinStop () {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    transServo.setPower(0);
                    return false;
                }
            };
        }
    }

    public static class Shoot {
        Servo servo;
        DcMotorEx motor;

        double oldPos;

        public Shoot(HardwareMap hardwareMap) {
            motor = hardwareMap.get(DcMotorEx.class, "ShooterMotor");
            servo = hardwareMap.get(Servo.class, "SpoonServo");
        }
        public Action holySpoonUp () {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    servo.setPosition(0.86);
                    packet.addLine("SpoonUp");
                    return false;
                }
            };
        }
        public Action holySpoonDown () {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    servo.setPosition(0.97);
                    packet.addLine("SpoonDown");
                    return false;
                }
            };
        }
        public Action shooterOn () {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    motor.setVelocity(shootVelocity);
                    return (motor.getVelocity() < shootVelocity);
                }
            };
        }
        public Action shooterOff () {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    motor.setPower(0);
                    return false;
                }
            };
        }
    }

    public static class Combined{
        Blue6BallFar.Shoot shoot;
        Blue6BallFar.Intake intake;

        public Combined(HardwareMap hardwareMap) {
            shoot = new Blue6BallFar.Shoot(hardwareMap);
            intake = new Blue6BallFar.Intake(hardwareMap);
        }

        public SequentialAction shoot1 () {
            return new SequentialAction(
                    shoot.holySpoonUp(),
                    new SleepAction(0.5),
                    shoot.holySpoonDown(),
                    new SleepAction(0.5),
                    intake.transSpinIn(),
                    new SleepAction(1),
                    intake.transSpinStop(),
                    intake.spinStop());
        }
    }
    public void runOpMode () {
        Pose2d initialPose = new Pose2d(62, 15, Math.PI);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Blue6BallFar.Shoot shoot = new Blue6BallFar.Shoot(hardwareMap);
        Blue6BallFar.Intake intake = new Blue6BallFar.Intake(hardwareMap);
        Blue6BallFar.Combined combined = new Blue6BallFar.Combined(hardwareMap);

        TrajectoryActionBuilder aim = drive.actionBuilder(initialPose)
                .lineToX(shootPosX)
                .turnTo(shootAngle);

        /*TrajectoryActionBuilder theivery = aim.endTrajectory().fresh()
                .turnTo(Math.PI/2)
                .splineToConstantHeading(new Vector2d(61, 60), Math.PI/2);

        TrajectoryActionBuilder getaway = drive.actionBuilder(new Pose2d(61, 60, Math.PI/2))
                .setTangent((Math.PI*2)-1.85)
                .splineToLinearHeading(new Pose2d(shootPosX, 15, shootAngle), (Math.PI/2)*3); */

        TrajectoryActionBuilder firsttwo = aim.endTrajectory().fresh()
                .setTangent(Math.PI)
                .splineToLinearHeading(new Pose2d(36, 30, Math.PI/2), Math.PI/2)
                .lineToY(54);

        TrajectoryActionBuilder end = firsttwo.endTrajectory().fresh()
                .setTangent(-Math.PI/2)
                .splineToLinearHeading(new Pose2d(55, 15, shootAngle), shootAngle-Math.PI);


        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        shoot.holySpoonDown(),
                        aim.build(),
                        shoot.shooterOn(),
                        combined.shoot1(),
                        combined.shoot1(),
                        combined.shoot1(),
                        shoot.shooterOff(),
                        intake.spinIn(),
                        intake.transSpinIn(),
                        firsttwo.build(),
                        end.build(),
                        intake.spinStop(),
                        intake.transSpinStop(),
                        shoot.shooterOn(),
                        combined.shoot1(),
                        combined.shoot1(),
                        combined.shoot1(),
                        shoot.shooterOff()
                        /*
                        intake.spinIn(),
                        intake.transSpinIn(),
                        theivery.build(),
                        new SleepAction(3),
                        getaway.build(),
                        intake.spinStop(),
                        intake.transSpinStop(),
                        shoot.shooterOn(),
                        combined.shoot1(),
                        combined.shoot1(),
                        combined.shoot1(),
                        shoot.shooterOff()*/
                )
        );
    }
}
