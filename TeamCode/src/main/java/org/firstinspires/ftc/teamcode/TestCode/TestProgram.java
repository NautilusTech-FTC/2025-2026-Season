package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/*
@TeleOp()
public class TestProgram extends OpMode {
    @Override
    public void init() {
        telemetry.addData("Hello", "World");
    }

    @Override
    public void loop() {

    }
}

@Autonomous()
public class TestProgram extends OpMode {
    @Override
    public void init() {
        telemetry.addData("Message", "Hello, Katie.");
    }

    public void loop() {

    }
}
*/

@TeleOp()
public class TestProgram extends OpMode {
    @Override
    public void init() {
        String myName = "Katie Chuo";
        int grade = 9;

        telemetry.addData("Driver", myName);
        telemetry.addData("Grade", grade);
        telemetry.update();
    }

    public void loop() {

    }
}
