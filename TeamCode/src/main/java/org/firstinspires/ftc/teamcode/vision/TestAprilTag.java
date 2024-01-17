package org.firstinspires.ftc.teamcode.vision;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

@TeleOp
public class TestAprilTag extends LinearOpMode {

    AprilTagProcessor tagProcessor1;
    AprilTagProcessor tagProcessor2;
    VisionPortal visionPortal1;
    VisionPortal visionPortal2;

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
                .setCameraResolution(new Size(1920, 1080))
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

        while (opModeIsActive()) {

            tagTelemtry();
        }
    }

    public void tagTelemtry(){
        tagProcessor1.setPoseSolver(AprilTagProcessor.PoseSolver.OPENCV_SOLVEPNP_EPNP);

        if (tagProcessor1.getDetections().size() > 0){

            ArrayList<AprilTagDetection> tag = tagProcessor1.getDetections();

            for (AprilTagDetection tags : tag){

                double PosicaoX, PosicaoY, Orientation;

                // daq em diante é tudo cordenada do roadrunner

                PosicaoX = tags.metadata.fieldPosition.get(0) + (tags.ftcPose.y *  Math.cos(Math.toRadians(tags.metadata.fieldPosition.get(3)))) +
                        (tags.ftcPose.x * Math.sin(Math.toRadians(tags.metadata.fieldPosition.get(3))));
                PosicaoY = tags.metadata.fieldPosition.get(1) + (tags.ftcPose.x * -Math.cos(Math.toRadians(tags.metadata.fieldPosition.get(3)))) +
                        (tags.ftcPose.x * Math.sin(Math.toRadians(tags.metadata.fieldPosition.get(3))));

                // Ponto original (x, y)
                double x = PosicaoX;
                double y = PosicaoY;
                // Ponto de rotação (h, k)
                double h = tags.metadata.fieldPosition.get(0);
                double k = tags.metadata.fieldPosition.get(1);
                // Ângulo de rotação em radianos
                double theta = Math.toRadians(tags.ftcPose.yaw);
                // Aplicar rotação
                double xRotacionado = (((x - h) * Math.cos(theta)) - ((y - k) * Math.sin(theta))) + h;
                double yRotacionado = (((x - h) * Math.sin(theta)) + ((y - k) * Math.cos(theta))) + k;


                Orientation = -tags.metadata.fieldPosition.get(3) + 180 + (tags.ftcPose.bearing + (Math.atan2(tags.ftcPose.x, tags.ftcPose.y))) + tags.ftcPose.yaw;

                telemetry.addLine("==========================");
                telemetry.addData("Detected tag ID", tags.id);
                telemetry.addData("Translation X", tags.ftcPose.x);
                telemetry.addData("Translation Y", tags.ftcPose.y);
                //telemetry.addData("Translation Z", tags.ftcPose.x);
                //telemetry.addLine("=============");
                telemetry.addData("Rotation Yaw", tags.ftcPose.yaw);
                //telemetry.addData("Rotation Pitch", tags.ftcPose.pitch);
                //telemetry.addData("Rotation Roll", tags.ftcPose.roll);
                telemetry.addLine("=============");
                telemetry.addData("Range", tags.ftcPose.range);
                telemetry.addData("Elevation", tags.ftcPose.elevation);
                telemetry.addData("Bearing", tags.ftcPose.bearing);
                telemetry.addLine("=============");
                telemetry.addData("Confidence:", tags.decisionMargin);
                telemetry.addData("FieldPos", tags.metadata.fieldPosition);

                telemetry.addData("RobotPosX:", PosicaoX);
                telemetry.addData("RobotPosY:", PosicaoY);
                telemetry.addData("Orientation:", Orientation);

            }
        }
        /*
        tagProcessor2.setPoseSolver(AprilTagProcessor.PoseSolver.OPENCV_SOLVEPNP_EPNP);

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
        telemetry.update();

    }

}
