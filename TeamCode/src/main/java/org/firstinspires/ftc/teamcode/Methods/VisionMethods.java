package org.firstinspires.ftc.teamcode.Methods;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Config
public class VisionMethods {

    DrivingMethods Drive = new DrivingMethods();

    double tagX;
    double correctionValue;
    boolean targetAcquired;
    int target;


    public static double divisor = 1;
    public static double blueOffset = -4;
    public static double redOffset = 0;

    private Position cameraPosition = new Position(DistanceUnit.INCH,
            3.004025, 7.06102, 8.85674, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    double x, y, z, yaw;

    double offsetIMU = 0;
    double IMUPreValue = 0;
    double IMUValue = 0;
    IMU imu;




    public void init(HardwareMap hardwareMap) {
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        aprilTag = new AprilTagProcessor.Builder()

                /* The following default settings are available to un-comment and edit as needed. */
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setCameraPose(cameraPosition, cameraOrientation)

                /* == CAMERA CALIBRATION ==
                If you do not manually specify calibration parameters, the SDK will attempt
                to load a predefined calibration for your camera. */
                .setLensIntrinsics(481.985, 481.985, 334.203, 241.948)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.setCameraResolution(new Size(640, 480));

        builder.addProcessor(aprilTag);

        visionPortal = builder.build();

        
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
    public void setTeam(team) {
        if (team == 0) {
            target = 20;
        } else {
            target = 24;
        }
    }

    public void updateCamera() {
        targetAcquired = false;
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());
        for (AprilTagDetection detection : currentDetections) {
            if ((detection.id == target) & (detection.metadata != null)) {
                pose = detection.robotPose;
                x = detection.robotPose.getPosition().x;
                y = detection.robotPose.getPosition().y; //fixes the weird pi/2 rotation
                z = detection.robotPose.getPosition().z;
                yaw = detection.robotPose.getOrientation().getYaw(AngleUnit.RADIANS) - Math.PI/2;

                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", x, y, z));
                targetAcquired = true;
            }
        }
    }

    
    public double aim(int team, boolean doLocalize, PinpointMethods PinPoint, Telemetry telemetry) {
        targetAcquired = false;
        if(doLocalize) {
            updateCamera();
            if(targetAcquired) {
                PinPoint.localize(pose)
            } else {
                return(2);
            }
        }

        PinPoint.update;

        telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", x, y, z));
        telemetry.addData("yaw", yaw);
        IMUValue = PinPoint.heading
        if (targetAcquired) {
            if (team == 0) {
                correctionValue = tanThroughXY(x,y,-60, -60, 3.004025);
            } else {
                correctionValue = tanThroughXY(x,y,-60, 60, 3.004025)-Math.PI;
            }
            telemetry.addData("desired angle", correctionValue);

            offsetIMU = yaw - IMUValue;
            correctionValue = (correctionValue-yaw)/divisor;
        } else {
            if (team == 0) {
                correctionValue = tanThroughXY(x,y,-60, 60, 3.004025);
            } else {
                correctionValue = tanThroughXY(x,y,60, 60, 3.004025)-Math.PI;
            }
            telemetry.addData("desired angle", correctionValue);

            correctionValue = (correctionValue-(offsetIMU+IMUValue))/divisor;
        }

        telemetry.addData("correctionValue", correctionValue);
        telemetry.addData("IMU value", IMUValue);
        telemetry.addData("IMU offset", offsetIMU);

        if (Math.abs(correctionValue) < 0.01) {
            correctionValue = 0;
        } else if (Math.abs(correctionValue) > 1) {
            correctionValue = Math.signum(correctionValue);
        }

        return(correctionValue);
    }

    public double tanThroughXY(
            double robotX,
            double robotY,
            double targetX,
            double targetY,
            double radius
    ) {
        double xdiff = (targetX-robotX);
        double ydiff = (targetY-robotY);
        return(Math.PI + Math.atan(ydiff/xdiff) - Math.acos(radius/(Math.sqrt((xdiff*xdiff)+(ydiff*ydiff)))));
    }
}
