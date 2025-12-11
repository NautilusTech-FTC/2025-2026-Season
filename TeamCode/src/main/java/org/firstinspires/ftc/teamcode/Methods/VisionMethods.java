package org.firstinspires.ftc.teamcode.Methods;



import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Config
public class VisionMethods {
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    DrivingMethods Drive = new DrivingMethods();

    double tagX;
    double correctionValue;
    boolean targetAcquired;
    int target;


    public static double divisor = 40;
    public static double blueOffset = 0;
    public static double redOffset = -2;





    public void init(HardwareMap hardwareMap) {
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(webcamName, aprilTagProcessor);

    }

    /*public void statsDisplay() {
        for (AprilTagDetection detection : aprilTagProcessor.getDetections()) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

        telemetry.update();
    }*/

    public double aim(int team, Telemetry telemetry) {
        targetAcquired = false;
        if (team == 0) {
            target = 20;
        } else {
            target = 24;
        }

        for (AprilTagDetection detection : aprilTagProcessor.getDetections()) {
            if ((detection.id == target) & (detection.metadata != null)) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("X", detection.ftcPose.x));
                tagX = detection.ftcPose.x;
                targetAcquired = true;
            }
        }

        if (targetAcquired) {
            if (team == 0) {
                correctionValue = (tagX + blueOffset) / divisor;
            } else {
                correctionValue = (tagX + redOffset) / divisor;
            }
        } else {
            correctionValue = 0;
            return(2);
        }

        if (Math.abs(correctionValue) < 0.1) {
            correctionValue = 0;
        } else if (Math.abs(correctionValue) > 1) {
            correctionValue = Math.signum(correctionValue);
        }
        telemetry.addData("correction value", correctionValue);

        telemetry.update();
        return(correctionValue);
    }
}
