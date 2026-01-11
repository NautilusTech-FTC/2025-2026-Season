package org.firstinspires.ftc.teamcode.Methods;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TransferMethods {

    CRServo transferServo;
    Servo spoonServo;

    public NormalizedColorSensor colorSensor;

    /* public DistanceSensor distanceSensor;
    public double distance; */

    public enum DetectedColor {
        PURPLE,
        GREEN,
        UNKNOWN
    }

    public void init (HardwareMap hardwareMap) {
        transferServo = hardwareMap.get(CRServo.class, "TransferServo");
        spoonServo = hardwareMap.get(Servo.class, "SpoonServo");
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "DistanceSensor");
        colorSensor.setGain(15);

        //distanceSensor = hardwareMap.get(DistanceSensor.class, "DistanceSensor");
    }

    public void servoPower (double power) {
        transferServo.setPower(power);
    }

    public void spoonPos (double position) {
        spoonServo.setPosition(position);
    }

    public DetectedColor getDetectedColor(Telemetry telemetry) {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        float red, blue, green;

        red = colors.red / colors.alpha;
        blue = colors.blue / colors.alpha;
        green = colors.green / colors.alpha;

        telemetry.addData("Red",red);
        telemetry.addData("Blue",blue);
        telemetry.addData("Green",green);

        /* Norm: .12, .14, .17
        Purple: .08, .16, .11
        Green: .05, .13, .17 */

        if (red < 0.1 && blue > 0.15 && green < 0.15) {
            return DetectedColor.PURPLE;
        } else if (red < 0.1 && blue < 0.15 && green > 0.15) {
            return DetectedColor.GREEN;
        } else return DetectedColor.UNKNOWN;
    }

}
