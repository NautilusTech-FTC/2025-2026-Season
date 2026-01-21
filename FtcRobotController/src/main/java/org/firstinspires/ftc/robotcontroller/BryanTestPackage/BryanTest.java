package org.firstinspires.ftc.robotcontroller.BryanTestPackage;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp
public class BryanTest extends OpMode {
    DcMotor IntakeSlowMotor;
    DcMotor MainIntakeMotor;

    @Override
    public void init() {
        IntakeSlowMotor = hardwareMap.get(DcMotor .class,"IntakeSlowMotor");
        IntakeSlowMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MainIntakeMotor = hardwareMap.get(DcMotor.class,"MainIntakeMotor");
    }

    @Override
    public void loop() {
        if(gamepad1.a) {
            IntakeSlowMotor.setPower(0.5);
        } else if (gamepad1.b) {
            IntakeSlowMotor.setPower(-0.5);
        } else {
            IntakeSlowMotor.setPower(0);
        }
        if (gamepad1.left_bumper) {
            MainIntakeMotor.setPower(1);
        } else if (gamepad1.right_bumper) {
            MainIntakeMotor.setPower(-1);
        } else {
            MainIntakeMotor.setPower(0);
        }

    }
}
