package org.firstinspires.ftc.teamcode.vision;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

public class InitPipes {

    AprilTagProcessor tagProcessor1;
    AprilTagProcessor tagProcessor2;
    VisionPortal visionPortal1;
    VisionPortal visionPortal2;
    TfodProcessor teamPropTFOD;

    private static final String TFOD_MODEL_ASSET = "model_20240115_155137.tflite";
    private static final String[] LABELS = {
            "Blue Cube", "Red Cube"
    };

    private static InitPipes instancia;

    private InitPipes() {
        // Inicialização da classe
    }

    public static InitPipes getInstancia() {
        if (instancia == null) {
            instancia = new InitPipes();
        }
        return instancia;
    }

    public void initVision(HardwareMap hardwareMap){
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
                .setLensIntrinsics(1416.97, 1416.97, 912.464, 532.303)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagCustomDatabase.getCenterStageLibrary())
                .build();

        teamPropTFOD = new TfodProcessor.Builder()
                .setMaxNumRecognitions(10)
                .setUseObjectTracker(true)
                .setTrackerMaxOverlap((float) 0.2)
                .setTrackerMinSize(16)
                .setModelAssetName(TFOD_MODEL_ASSET)
                .setModelLabels(LABELS)
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
                .addProcessors(tagProcessor2, teamPropTFOD)
                .setCameraResolution(new Size(1920, 1080))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .setLiveViewContainerId(portalsList[1])
                .setAutoStopLiveView(true)
                .build();



        visionPortal1.setProcessorEnabled(tagProcessor1, true);
        visionPortal2.setProcessorEnabled(tagProcessor2, true);
        visionPortal2.setProcessorEnabled(teamPropTFOD, true);

        tagProcessor1.setPoseSolver(AprilTagProcessor.PoseSolver.OPENCV_SOLVEPNP_EPNP);
        tagProcessor2.setPoseSolver(AprilTagProcessor.PoseSolver.OPENCV_SOLVEPNP_EPNP);
    }

    public void closeCams(){
        visionPortal1.stopLiveView();
        visionPortal1.stopStreaming();
        visionPortal1.close();

        visionPortal2.stopLiveView();
        visionPortal2.stopStreaming();
        visionPortal2.close();

    }
    
    public AprilTagProcessor returnTagProcessor1(){
        return tagProcessor1;
    }
    public AprilTagProcessor returnTagProcessor2(){
        return tagProcessor2;
    }
    public VisionPortal returnVisionPortal1(){
        return visionPortal1;
    }
    public VisionPortal returnVisionPortal2(){
        return visionPortal2;
    }
    public TfodProcessor returnTFOD(){
        return teamPropTFOD;
    }

    public void activateTFODProcessor(boolean state){
        visionPortal2.setProcessorEnabled(teamPropTFOD, state);
    }
}
