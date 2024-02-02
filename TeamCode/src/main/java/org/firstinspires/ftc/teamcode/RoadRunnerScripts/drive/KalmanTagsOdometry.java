package org.firstinspires.ftc.teamcode.RoadRunnerScripts.drive;

import static org.firstinspires.ftc.teamcode.RoadRunnerScripts.drive.DriveConstants.encoderTicksToInches;

import android.util.Size;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.AprilTagCustomDatabase;
import org.firstinspires.ftc.teamcode.vision.mapper.KalmanPose;
import org.firstinspires.ftc.teamcode.vision.mapper.MecanumKinematics;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

public class KalmanTagsOdometry implements Localizer {

    AprilTagProcessor tagProcessor1;
    AprilTagProcessor tagProcessor2;
    VisionPortal visionPortal1;
    VisionPortal visionPortal2;

    List<com.arcrobotics.ftclib.geometry.Pose2d> measurePoses;
    List<Double> wheelsVels = new ArrayList<>();
    List<Double> TickVels = new ArrayList<>();

    String[] colors = {
            "red",
            "green",
            "blue",
            "yellow",
            "cyan",
            "magenta",
            "orange",
            "purple",
            "gray",
            "black"
    };

    SampleMecanumDrive drive;
    KalmanPose kalmanPose;

    public KalmanTagsOdometry(HardwareMap hardwareMap) {
        new KalmanTagsOdometry(hardwareMap, true);
    }

    public KalmanTagsOdometry(HardwareMap hardwareMap, boolean resetPos) {
        measurePoses = new ArrayList<>();
        kalmanPose = new KalmanPose(new com.arcrobotics.ftclib.geometry.Pose2d(0, 0, new Rotation2d(0)));
        drive = new SampleMecanumDrive(hardwareMap);

        int[] portalsList = VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL);

        tagProcessor1 = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setLensIntrinsics(1219.057, 1220.919, 617.782, 339.444)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagCustomDatabase.getCenterStageLibrary())
                .build();

        tagProcessor2 = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setLensIntrinsics(1494.943, 1490.958, 934.017, 549.413)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagCustomDatabase.getCenterStageLibrary())
                .build();

        visionPortal1 = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(tagProcessor1)
                .setCameraResolution(new Size(1280, 720))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .setLiveViewContainerId(portalsList[0])
                .setAutoStopLiveView(true)
                .build();

        visionPortal2 = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"))
                .addProcessor(tagProcessor2)
                .setCameraResolution(new Size(1920, 1080))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .setLiveViewContainerId(portalsList[1])
                .setAutoStopLiveView(true)
                .build();

        visionPortal1.setProcessorEnabled(tagProcessor1, true);
        visionPortal2.setProcessorEnabled(tagProcessor2, true);

        tagProcessor1.setPoseSolver(AprilTagProcessor.PoseSolver.OPENCV_SOLVEPNP_EPNP);
        tagProcessor2.setPoseSolver(AprilTagProcessor.PoseSolver.OPENCV_SOLVEPNP_EPNP);
    }

    @NonNull
    @Override
    public Pose2d getPoseEstimate() {

        /*
        if (tagProcessor1.getDetections().size() > 0){
            ArrayList<AprilTagDetection> tags = tagProcessor1.getDetections();
            for(AprilTagDetection tag : tags){

                com.arcrobotics.ftclib.geometry.Pose2d rpose = Positioner.getRobotPose(tag, new Transform2d(new Translation2d(-6.5, 0), new Rotation2d(Math.toRadians(180))));
                measurePoses.add(rpose);

                }
        }

        if (tagProcessor2.getDetections().size() > 0){

            ArrayList<AprilTagDetection> tags = tagProcessor2.getDetections();

            for (AprilTagDetection tag : tags){

                com.arcrobotics.ftclib.geometry.Pose2d rpose = Positioner.getRobotPose(tag, new Transform2d(new Translation2d(-6.5, 0), new Rotation2d(Math.toRadians(0))));
                measurePoses.add(rpose);

            }
        }
         */

        //Obtenção da velocidade do robo(vel)
        TickVels = drive.getWheelVelocities();
        for (double vels : TickVels){
            wheelsVels.add(encoderTicksToInches(vels));
        }
        com.arcrobotics.ftclib.geometry.Pose2d vel = MecanumKinematics.wheelToVel(wheelsVels.get(0), wheelsVels.get(3), wheelsVels.get(1), wheelsVels.get(2), 15.15749, 13.3859);
        //Drive: FL, Bl, BR, FR
        //wheelToVel: FL, FR, BL, BR

        //Obtenção da posição estimada do roadrunner(statePose)
        com.acmerobotics.roadrunner.geometry.Pose2d RoadrunnerStatePose = drive.getPoseEstimate();
        com.arcrobotics.ftclib.geometry.Pose2d statePose = new com.arcrobotics.ftclib.geometry.Pose2d(RoadrunnerStatePose.getX(), RoadrunnerStatePose.getY(), new Rotation2d(RoadrunnerStatePose.getHeading()));

        //Chama o fitro e passa os valores (vel, statePose, measurePoses)
        com.arcrobotics.ftclib.geometry.Pose2d filteredPose = kalmanPose.updateFilter(vel, measurePoses, 0);

        Pose2d estimatePose = new Pose2d(filteredPose.getX(), filteredPose.getY(), filteredPose.getHeading());

        return estimatePose;
    }

    @Override
    public void setPoseEstimate(@NonNull Pose2d pose2d) {
        drive.setPoseEstimate(pose2d);
    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        //Obtenção da velocidade do robo(vel)
        TickVels = drive.getWheelVelocities();
        for (double vels : TickVels){
            wheelsVels.add(encoderTicksToInches(vels));
        }
        com.arcrobotics.ftclib.geometry.Pose2d vel = MecanumKinematics.wheelToVel(wheelsVels.get(0), wheelsVels.get(3), wheelsVels.get(1), wheelsVels.get(2), 15.15749, 13.3859);
        //Drive: FL, Bl, BR, FR
        //wheelToVel: FL, FR, BL, BR

        Pose2d poseVelocity = new Pose2d(vel.getX(), vel.getY(), vel.getHeading());

        return poseVelocity;
    }

    @Override
    public void update() {

    }
}
