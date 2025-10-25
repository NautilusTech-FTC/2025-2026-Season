package org.firstinspires.ftc.teamcode.Methods;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.Methods.DrivingMethods;
import org.firstinspires.ftc.teamcode.Methods.AutonomousMethods;

@Autonomous
public class Meet0Auto extends OpMode {
    DrivingMethods drive = new DrivingMethods();
    AutonomousMethods auto = new AutonomousMethods(); // makes copies of the method classes with the functions we need so we can use them

    @Override
    public void init() {
        auto.init(hardwareMap);

    }

    @Override
    public void start() {
        resetRuntime();

        while (getRuntime() < 2.5) {
            auto.setDriveAngle(); // the robot moves forward at half-speed
        }
        auto.stopMotors();
        telemetry.addData("Stage:","done");
    }

    @Override
    public void loop() {
    } //this loop does nothing!! ; )

    @Override
    public void stop() {
        auto.stopMotors();

    } // makes the motors stop when you push stop
}
