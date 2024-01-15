package org.firstinspires.ftc.teamcode.vision;


import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.mapper.Positioner;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

@TeleOp
public class ApriltagOdometryTest extends LinearOpMode {

    AprilTagProcessor tagProcessor1;
    AprilTagProcessor tagProcessor2;
    VisionPortal visionPortal1;
    VisionPortal visionPortal2;

    FtcDashboard dashboard;
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
    @Override
    public void runOpMode() {

        int[] portalsList = VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL);

        tagProcessor1 = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setLensIntrinsics(822.317, 822.317, 319.495, 242.502)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagCustomDatabase.getCenterStageLibrary())
                .build();

        tagProcessor2 = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setLensIntrinsics(854.2712445, 875.96651965, 613.29710682, 537.91085293)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagCustomDatabase.getCenterStageLibrary())
                .build();

        visionPortal1 = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(tagProcessor1)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .setLiveViewContainerId(portalsList[0])
                .setAutoStopLiveView(true)
                .build();

        visionPortal2 = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"))
                .addProcessor(tagProcessor2)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .setLiveViewContainerId(portalsList[1])
                .setAutoStopLiveView(true)
                .build();

        visionPortal1.setProcessorEnabled(tagProcessor1, true);
        visionPortal2.setProcessorEnabled(tagProcessor2, true);

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();

        tagProcessor1.setPoseSolver(AprilTagProcessor.PoseSolver.OPENCV_SOLVEPNP_EPNP);
        tagProcessor2.setPoseSolver(AprilTagProcessor.PoseSolver.OPENCV_SOLVEPNP_EPNP);
        dashboard = FtcDashboard.getInstance();

        while (opModeIsActive()) {
            tagTelemtry();
        }
    }

    public void tagTelemtry(){
        if (tagProcessor1.getDetections().size() > 0){
            TelemetryPacket packet = new TelemetryPacket();
            ArrayList<AprilTagDetection> tags = tagProcessor1.getDetections();
            for(AprilTagDetection tag : tags){

                Pose2d rpose = Positioner.tagToCamPose(tag);
                packet.put("tag X", tag.ftcPose.x);
                packet.put("tag Y", tag.ftcPose.y);
                packet.put("tag rot", tag.ftcPose.yaw);
                packet.put("tag teoreticalpose", Positioner.tagTheoreticalPose(tag));

                //Positioner.getRobotPose(tags.get(0), new Transform2d(new Translation2d(0, 6.5), new Rotation2d(0)));

                packet.put("pose ", rpose);

                packet.fieldOverlay()
                        .setStroke(colors[tag.id])
                        .strokeCircle(rpose.getX(), rpose.getY(), 10)
                        .strokeLine(rpose.getX(), rpose.getY(), rpose.getX() + 10*rpose.getRotation().getCos(), rpose.getY()+ 10*rpose.getRotation().getSin());
            }
            dashboard.sendTelemetryPacket(packet);
        }
        /*
        if (tagProcessor2.getDetections().size() > 0){

            ArrayList<AprilTagDetection> tag = tagProcessor2.getDetections();

            for (AprilTagDetection tags : tag){
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
                telemetry.addData("Bearing", tags.ftcPose.bearing);
                telemetry.addLine("=============");
                telemetry.addData("FieldPos", tags.metadata.fieldPosition);
                telemetry.addData("RobotPosX:", tags.metadata.fieldPosition.get(0) + tags.ftcPose.y);
                telemetry.addData("RobotPosY:", tags.metadata.fieldPosition.get(1) + tags.ftcPose.x);
                telemetry.addData("Orientation:", -tags.metadata.fieldPosition.get(3) + 180 + (tags.ftcPose.bearing + (Math.atan(tags.ftcPose.x/tags.ftcPose.y))));
            }
        }
        */
        //telemetry.update();

    }

}
