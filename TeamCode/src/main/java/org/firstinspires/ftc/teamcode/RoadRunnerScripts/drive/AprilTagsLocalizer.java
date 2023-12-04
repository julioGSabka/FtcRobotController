package org.firstinspires.ftc.teamcode.RoadRunnerScripts.drive;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.vision.AprilTagCustomDatabase;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

@Config
public class AprilTagsLocalizer implements Localizer {

    private Pose2d poseOffset = new Pose2d();
    private static Pose2d mPoseEstimate = new Pose2d();
    private Pose2d rawPose = new Pose2d();

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
        poseOffset = new Pose2d();
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

        tagProcessor.setPoseSolver(AprilTagProcessor.PoseSolver.OPENCV_SOLVEPNP_EPNP);

        if (tagProcessor.getDetections().size() > 0) {

            tag = tagProcessor.getDetections();

            for (org.firstinspires.ftc.vision.apriltag.AprilTagDetection tags : tag) {
                /*
                telemetry.addData("Detected tag ID", tags.id);
                telemetry.addData("Translation X", tags.ftcPose.x);
                telemetry.addData("Translation Y", tags.ftcPose.y);
                telemetry.addData("Translation Z", tags.ftcPose.x);
                telemetry.addLine("=============");
                telemetry.addData("Rotation Yaw", tags.ftcPose.yaw);
                telemetry.addData("Rotation Pitch", tags.ftcPose.pitch);
                telemetry.addData("Rotation Roll", tags.ftcPose.roll);
                telemetry.addLine("=============");
                telemetry.addData("Range", tags.ftcPose.range);
                telemetry.addData("Elevation", tags.ftcPose.elevation);
                telemetry.addData("Bearing", Math.toRadians(tags.ftcPose.bearing));
                telemetry.addLine("=============");
                telemetry.addData("FieldPos", tags.metadata.fieldPosition);
                telemetry.addData("Tentativa X", tags.metadata.fieldPosition.get(0) - tags.ftcPose.y);
                */
            }
        }

    }

    @Override
    public Pose2d getPoseEstimate() {

        tagProcessor.setPoseSolver(AprilTagProcessor.PoseSolver.OPENCV_SOLVEPNP_EPNP);

        if (tagProcessor.getDetections().size() > 0){

            tag = tagProcessor.getDetections();

            for (AprilTagDetection tags : tag){
                VectorF targetPose = tags.metadata.fieldPosition;
                AprilTagPoseFtc camToTarget = tags.ftcPose;


            }
        }

    }

    private final List<State> m_states;
    public Pose2d poseMeters;


    public Trajectory transformBy(Transform2d transform) {
        var firstState = m_states.get(0);
        Pose2d firstPose = firstState.poseMeters;

        // Calculate the transformed first pose.
        Pose2d newFirstPose = firstPose.plus(transform);
        List<State> newStates = new ArrayList<>();

        newStates.add(
            new State(
                firstState.timeSeconds,
                firstState.velocityMetersPerSecond,
                firstState.accelerationMetersPerSecondSq,
                newFirstPose,
                firstState.curvatureRadPerMeter));

        for (int i = 1; i < m_states.size(); i++) {
            var state = m_states.get(i);
            // We are transforming relative to the coordinate frame of the new initial pose.
            newStates.add(
                new State(
                    state.timeSeconds,
                    state.velocityMetersPerSecond,
                    state.accelerationMetersPerSecondSq,
                    newFirstPose.plus(state.poseMeters.minus(firstPose)),
                    state.curvatureRadPerMeter));
        }

        return new Trajectory(newStates);
    }

}