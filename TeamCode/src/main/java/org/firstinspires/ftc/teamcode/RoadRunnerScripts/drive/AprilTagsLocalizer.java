/*
package org.firstinspires.ftc.teamcode.RoadRunnerScripts.drive;

import static com.acmerobotics.roadrunner.util.Angle.norm;

import android.util.Size;

import androidx.annotation.Nullable;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.AprilTagCustomDatabase;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.jetbrains.annotations.NotNull;

import java.util.ArrayList;

@Config
public class AprilTagsLocalizer implements Localizer {

    private static Pose2d mPoseEstimate = new Pose2d();
    private Pose2d rawPose = new Pose2d();
    private double PoseX = 0;
    private double PoseY = 0;
    private double Orientation = 0;
    private int BestTag = 0;

    public static T265Camera slamra;

    private ArrayList<AprilTagDetection> detections = new ArrayList<AprilTagDetection>();

    AprilTagProcessor tagProcessor;
    VisionPortal visionPortal;
    ArrayList<AprilTagDetection> tag;

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

        mPoseEstimate = new Pose2d();
        rawPose = new Pose2d();

        //Declara e starta a câmera
        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setLensIntrinsics(822.317, 822.317, 319.495, 242.502)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagCustomDatabase.getCenterStageLibrary())
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(tagProcessor)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();

        visionPortal.setProcessorEnabled(tagProcessor, true);

        if (slamra == null) {
            slamra = new T265Camera(new Transform2d(new Translation2d(0.045,-0.08), new Rotation2d(Math.toRadians(0))), 0, hardwareMap.appContext);
            RobotLog.d("Created Realsense Object");
        }
        try {
            startRealsense();
        } catch (Exception ignored) {
            RobotLog.v("Realsense already started");
            if (resetPos) {
                slamra.setPose(new com.arcrobotics.ftclib.geometry.Pose2d(0,0, new Rotation2d(0)));
            }
        }
        if (slamra.getLastReceivedCameraUpdate().confidence == T265Camera.PoseConfidence.Failed) {
            RobotLog.e("Realsense Failed to get Position");
        }

    }

    @NotNull
    @Override
    public Pose2d getPoseEstimate() {

        tagProcessor.setPoseSolver(AprilTagProcessor.PoseSolver.OPENCV_SOLVEPNP_EPNP);

        if (tagProcessor.getDetections().size() > 0){

            tag = tagProcessor.getDetections();
            int detectionNumber = tag.size();

            if (detectionNumber > 1) {
                BestTag = tag.get(0).id; // Inicializa BestTag com o primeiro elemento

                for (int i = 1; i < detectionNumber; i++) {
                    if (tag.get(i).decisionMargin > tag.get(i - 1).decisionMargin) {
                        BestTag = tag.get(i).id;
                    }
                }
            } else {
                BestTag = (detectionNumber == 1) ? tag.get(0).id : 0;
            }

            PoseX = tag.get(BestTag).metadata.fieldPosition.get(0) + tag.get(BestTag).ftcPose.y;
            PoseY = tag.get(BestTag).metadata.fieldPosition.get(1) + tag.get(BestTag).ftcPose.x;
            Orientation = -tag.get(BestTag).metadata.fieldPosition.get(3) + 180 + (tag.get(BestTag).ftcPose.bearing + (Math.atan(tag.get(BestTag).ftcPose.x / tag.get(BestTag).ftcPose.y)));

            //The T265's unit of measurement is meters. Dividing it by .0254 converts meters to inches.
            rawPose = new com.acmerobotics.roadrunner.geometry.Pose2d(PoseX /.0254, PoseY / .0254, norm(Math.toRadians(Orientation)));
            mPoseEstimate = rawPose; //offsets the pose to be what the pose estimate is;

        } else {
            RobotLog.v("NULL Tags Detections");
        }

        return mPoseEstimate;
    }

    @Override
    public void setPoseEstimate(@NotNull Pose2d pose2d) {
        slamra.setPose(new com.arcrobotics.ftclib.geometry.Pose2d(new Translation2d(pose2d.getX() * .0254, pose2d.getY() * .0254), new Rotation2d(norm(pose2d.getHeading()))));
        RobotLog.v("Set Pose to " + pose2d.toString());
    }


    public static double getHeading() {
        return mPoseEstimate.getHeading();
    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        return null;
    }

    @Override
    public void update() {
        up = slamra.getLastReceivedCameraUpdate();
        poseConfidence = up.confidence;

    }

    private double norm(double angle) {
        while (angle>Math.toRadians(360)) angle-=Math.toRadians(360);
        while (angle<=0) angle+=Math.toRadians(360);
        return angle;
    }
}
*/