package org.firstinspires.ftc.robotcontroller.BryanTestPackage;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Set;


@TeleOp
public class BryanTest extends OpMode {
    DcMotor IntakeSlowMotor;
    DcMotor MainIntakeMotor;
    DcMotor CounterSpinMotor;
    CRServo CrServo0;
    CRServo CrServo1;
    Servo RingServo;

    @Override
    public void init() {
        IntakeSlowMotor = hardwareMap.get(DcMotor.class, "IntakeSlowMotor");
        IntakeSlowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        IntakeSlowMotor.setTargetPosition(0);
        IntakeSlowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        IntakeSlowMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MainIntakeMotor = hardwareMap.get(DcMotor.class, "MainIntakeMotor");
        CounterSpinMotor = hardwareMap.get(DcMotor.class, "CounterSpinMotor");
        CrServo0 = hardwareMap.get(CRServo.class, "CrServo0");
        CrServo1 = hardwareMap.get(CRServo.class, "CrServo1");
        RingServo = hardwareMap.get(Servo.class, "RingServo");

    }

    @Override
    public void loop() {

        if (gamepad1.a) {
            IntakeSlowMotor.setTargetPosition(100);
        } else if (gamepad1.b) {
            IntakeSlowMotor.setTargetPosition(200);
        } else {
            IntakeSlowMotor.setPower(0);
        }
        if (gamepad1.right_bumper) {
            MainIntakeMotor.setPower(1);
            CounterSpinMotor.setPower(1);
            CrServo1.setPower(-1);
            CrServo0.setPower(-1);
        } else if (gamepad1.left_bumper) {
            CounterSpinMotor.setPower(-1);
            MainIntakeMotor.setPower(-1);
            CrServo1.setPower(1);
            CrServo0.setPower(1);
        } else {
            MainIntakeMotor.setPower(0);
            CounterSpinMotor.setPower(0);
            CrServo1.setPower(0);
            CrServo0.setPower(0);
        }
        if (gamepad1.y & getRuntime()>0.6) {
            resetRuntime();
            RingServo.setPosition(0.94);
        }
        if (getRuntime() > 0.3) {
            RingServo.setPosition(1);
        }

        telemetry.addData("Runtime",getRuntime());
        telemetry.addData("IntakePos",IntakeSlowMotor.getCurrentPosition());
    }
}