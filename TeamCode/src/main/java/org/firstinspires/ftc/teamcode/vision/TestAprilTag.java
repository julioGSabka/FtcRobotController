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

    @Override
    public void runOpMode() {

        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setLensIntrinsics(822.317, 822.317, 319.495, 242.502)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagCustomDatabase.getCenterStageLibrary())
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(tagProcessor)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();

        visionPortal.setProcessorEnabled(tagProcessor, true);

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            tagProcessor.setPoseSolver(AprilTagProcessor.PoseSolver.OPENCV_SOLVEPNP_EPNP);

            if (tagProcessor.getDetections().size() > 0){

                ArrayList<AprilTagDetection> tag = tagProcessor.getDetections();


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
                    telemetry.addData("Bearing", Math.toRadians(tags.ftcPose.bearing));
                    telemetry.addLine("=============");
                    telemetry.addData("FieldPos", tags.metadata.fieldPosition);
                    telemetry.addData("Tentativa X", tags.metadata.fieldPosition.get(0) - tags.ftcPose.y);

                }
            }

            telemetry.update();

        }
    }
}
