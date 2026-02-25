package org.firstinspires.ftc.teamcode.Methods;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
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
    boolean localized = false;
    boolean doLocalize = true;


    public static double divisor = 1;
    public static double blueGoalX = -60;
    public static double blueGoalY = -60;
    public static double redGoalX = -60;
    public static double redGoalY = 60;

    public double targetX = -60;
    public double targetY = -60;



    private Position cameraPosition = new Position(DistanceUnit.INCH,
            3.004025, 7.06102, 8.85674, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    double cameraX, cameraY, cameraZ, cameraYaw;

    double offsetIMU = 0;
    double IMUPreValue = 0;
    double IMUValue = 0;
    IMU imu;

    Pose3D cameraPose;
    Pose2D robotPose;




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

    public void setTeam(int team) {
        if (team == 0) {
            target = 20;
            targetX = blueGoalX;
            targetY = blueGoalY;
        } else {
            target = 24;
            targetX = redGoalX;
            targetY = redGoalY;
        }
    }

    public void reLocalize() {
        doLocalize = true;
    }

    public void updateCamera(Telemetry telemetry) {
        targetAcquired = false;
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if ((detection.id == target) & (detection.metadata != null)) {
                cameraPose = detection.robotPose;
                cameraX = detection.robotPose.getPosition().x;
                cameraY = detection.robotPose.getPosition().y;
                cameraZ = detection.robotPose.getPosition().z;
                cameraYaw = detection.robotPose.getOrientation().getYaw(AngleUnit.RADIANS);

                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", cameraX, cameraY, cameraZ));
                if(cameraZ < 10) {
                    targetAcquired = true;
                }
            }
        }
    }

    
    public double aim(boolean enable, PinpointMethods PinPoint, Telemetry telemetry) {
        if(doLocalize) {
            updateCamera(telemetry);
            if(targetAcquired) {
                PinPoint.localize(new Pose2D(DistanceUnit.INCH, cameraX, cameraY, AngleUnit.RADIANS, cameraYaw));
                localized = true;
                doLocalize = false;
            } else {
                return(2);
            }
        }
        if (!localized) {
            return(2);
        }
        if (!enable) {
            return(3);
        }

        PinPoint.update();
        telemetry.addData("yaw", cameraYaw);
        robotPose = PinPoint.pose();

        correctionValue = tanThroughXY(robotPose.getX(DistanceUnit.INCH),robotPose.getY(DistanceUnit.INCH),targetX, targetY, 3.004025);

        telemetry.addData("desired angle", correctionValue);

        correctionValue = (correctionValue-(offsetIMU+IMUValue))/divisor;


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
