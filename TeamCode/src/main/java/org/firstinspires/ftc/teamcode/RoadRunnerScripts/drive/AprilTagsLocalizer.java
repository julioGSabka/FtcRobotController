/*
package org.firstinspires.ftc.teamcode.RoadRunnerScripts.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Pipelines.AprilTagDetectionPipeline;
import org.jetbrains.annotations.NotNull;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

@Config
public class AprilTagsLocalizer implements Localizer {

    private Pose2d poseOffset = new Pose2d();
    private static Pose2d mPoseEstimate = new Pose2d();
    private Pose2d rawPose = new Pose2d();

    private ArrayList<AprilTagDetection> detections = new ArrayList<AprilTagDetection>();


    OpenCvWebcam webcam;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    //Intrinsecos da camera
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    double tagsize = 0.127;

    int numFramesWithoutDetection = 0;

    final float DECIMATION_HIGH = 3;
    final float DECIMATION_LOW = 2;
    final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;

    public AprilTagsLocalizer(HardwareMap hardwareMap) {
        new AprilTagsLocalizer(hardwareMap, true);
    }

    public AprilTagsLocalizer(HardwareMap hardwareMap, boolean resetPos) {
        poseOffset = new Pose2d();
        mPoseEstimate = new Pose2d();
        rawPose = new Pose2d();

        //Declara e starta a câmera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        webcam.setPipeline(aprilTagDetectionPipeline);

        webcam.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        detections = aprilTagDetectionPipeline.getDetectionsUpdate();

        if (detections.size() == 0) {
            numFramesWithoutDetection++;

            if (numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION) {
                aprilTagDetectionPipeline.setDecimation(DECIMATION_LOW);
            }
        }
        // We do see tags!
        else {
            numFramesWithoutDetection = 0;

            if (detections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS) {
                aprilTagDetectionPipeline.setDecimation(DECIMATION_HIGH);
            }
        }


        for (AprilTagDetection detection : detections) {
            Orientation rot = Orientation.getOrientation(detection.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);
            /*
            telemetry.addData("Detected tag ID= ", detection.id);
            telemetry.addData("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER);
            telemetry.addData("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER);
            telemetry.addData("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER);
            telemetry.addData("Rotation Yaw: %.2f degrees", rot.firstAngle);
            telemetry.addData("Rotation Pitch: %.2f degrees", rot.secondAngle);
            telemetry.addData("Rotation Roll: %.2f degrees", rot.thirdAngle);
             */
/*
import org.jetbrains.annotations.NotNull;

        @NotNull
        @Override
        public Pose2d getPoseEstimate() {

            for (AprilTagDetection detection : detections) {
                Orientation rot = Orientation.getOrientation(detection.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);

                if (detection.id == 10) {

                }
                if (detection.id == 7) {

                }
            }
            //variable up is updated in update()

            //The FTC265 library uses Ftclib geometry, so I need to convert that to road runner Geometry
            //TODO: convert all Ftclib geometry to ACME robotics geometry in T265Camera.java
        /*
        if (up != null) {
            Translation2d oldPose = up.pose.getTranslation();
            Rotation2d oldRot = up.pose.getRotation();
            //The T265's unit of measurement is meters.  dividing it by .0254 converts meters to inches.
            rawPose = new Pose2d(oldPose.getX() / .0254, oldPose.getY() / .0254, norm(oldRot.getRadians())); //raw pos
            mPoseEstimate = rawPose; //offsets the pose to be what the pose estimate is;
        } else {
            RobotLog.v("NULL Camera Update");
        }

        return mPoseEstimate;
        */

       // }
  //  }
//}
