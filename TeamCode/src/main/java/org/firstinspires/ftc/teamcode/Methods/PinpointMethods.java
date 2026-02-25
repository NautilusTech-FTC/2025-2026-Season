package org.firstinspires.ftc.teamcode.Methods;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import com.qualcomm.robotcore.hardware.HardwareMap;

//fern
@Config
public class PinpointMethods {
    GoBildaPinpointDriver PinPoint;
    public static double parYTicks = -1868.305195101019; // y position of the parallel encoder (in tick units)
    public static double perpXTicks = -3441.154550441893; // x position of the perpendicular encoder (in tick units)
    public static double inPerTick = 0.00209071;

    Pose2D pos;
    public void init (HardwareMap hardwareMap) {
        PinPoint = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        double mmPerTick = inPerTick * 25.4;
        PinPoint.setEncoderResolution(1 / mmPerTick, DistanceUnit.MM);
        PinPoint.setOffsets(mmPerTick*parYTicks, mmPerTick*perpXTicks, DistanceUnit.MM);
        PinPoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        PinPoint.resetPosAndIMU();
    }

    public void update() {
        PinPoint.update();
        pos = PinPoint.getPosition();
    }
    
    public Pose2D pose() {
        return pos;
    }

    public void localize(Pose2D position) {
        PinPoint.setPosition(position);
    }
}
