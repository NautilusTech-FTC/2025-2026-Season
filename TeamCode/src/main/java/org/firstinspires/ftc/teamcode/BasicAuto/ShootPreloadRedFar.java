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
public class ShootPreloadRedFar extends LinearOpMode {
    public static int shootPosX = 55;
    public static double shootAngle = 2.85;
    public static double shootVelocity = 1500;


    public static class Intake {
        private DcMotor motor;
        private CRServo servo;

        public Intake (HardwareMap hardwareMap) {
            motor = hardwareMap.get(DcMotor.class, "IntakeMotor");
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            servo = hardwareMap.get(CRServo.class, "TransferServo");
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
                    servo.setPower(-1);
                    packet.addLine("TransSpinIn");
                    return false;
                }
            };
        }
        public Action transSpinOut () {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    servo.setPower(1);
                    return false;
                }
            };
        }
        public Action transSpinStop () {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    servo.setPower(0);
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
                    servo.setPosition(0.8);
                    packet.addLine("SpoonUp");
                    return false;
                }
            };
        }
        public Action holySpoonDown () {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    servo.setPosition(1);
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
    public void runOpMode () {
        Pose2d initialPose = new Pose2d(62, -15, Math.PI);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        ShootPreloadBlueFar.Shoot shoot = new ShootPreloadBlueFar.Shoot(hardwareMap);
        ShootPreloadBlueFar.Intake intake = new ShootPreloadBlueFar.Intake(hardwareMap);

        TrajectoryActionBuilder aim = drive.actionBuilder(initialPose)
                .lineToX(shootPosX)
                .turnTo(shootAngle);

        TrajectoryActionBuilder exit = drive.actionBuilder(initialPose)
                .turnTo(Math.PI)
                .lineToX(39);

        waitForStart();



        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        aim.build(),
                        shoot.shooterOn(),
                        new SequentialAction(
                                shoot.holySpoonUp(),
                                new SleepAction(0.5),
                                shoot.holySpoonDown(),
                                new SleepAction(0.5),
                                intake.transSpinIn(),
                                new SleepAction(1),
                                intake.transSpinStop(),
                                intake.spinStop()
                        ),
                        new SequentialAction(
                                shoot.holySpoonUp(),
                                new SleepAction(0.5),
                                shoot.holySpoonDown(),
                                new SleepAction(0.5),
                                intake.transSpinIn(),
                                new SleepAction(1),
                                intake.transSpinStop(),
                                intake.spinStop()
                        ),
                        new SequentialAction(
                                shoot.holySpoonUp(),
                                new SleepAction(0.5),
                                shoot.holySpoonDown(),
                                new SleepAction(0.5),
                                intake.transSpinIn(),
                                new SleepAction(1),,
                                intake.spinStop()
                                intake.transSpinStop()
                        ),
                        shoot.shooterOff(),
                        exit.build()
                )
        );
    }
}
