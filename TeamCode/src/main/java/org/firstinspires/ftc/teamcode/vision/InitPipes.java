package org.firstinspires.ftc.teamcode.vision;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.FocusControl;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;
@Config
public class InitPipes {

    boolean targetFound = false;
    AprilTagDetection desiredTag = null;
    int desiredBlueTagID = -1;
    int desiredRedTagID = -1;

    static double DESIRED_DISTANCE = 10.0;
    static double SPEED_GAIN  =  -0.02  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    static double STRAFE_GAIN =  0.03 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    static double TURN_GAIN   =  -0.01  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    static double MAX_AUTO_SPEED  = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    static double MAX_AUTO_STRAFE = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    static double MAX_AUTO_TURN   = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    double drive = 0;    // Desired forward power/speed (-1 to +1)
    double strafe = 0;   // Desired strafe power/speed (-1 to +1)
    double turn = 0;     // Desired turning power/speed (-1 to +1)

    static double exposureMS = 5;
    static double focusLength = 0;


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
                .setLensIntrinsics(854.2712445, 875.96651965, 613.29710682, 537.91085293)
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

        while(visionPortal1.getCameraState() != VisionPortal.CameraState.STREAMING);
        while(visionPortal2.getCameraState() != VisionPortal.CameraState.STREAMING);

        setCamSettings();
    }

    public ArrayList<AprilTagDetection> updateTagProcessor1(){
        return tagProcessor1.getDetections();
    }
    public ArrayList<AprilTagDetection> updateTagProcessor2(){
        return tagProcessor2.getDetections();
    }

    public void closeCams(){
        visionPortal1.stopLiveView();
        visionPortal1.stopStreaming();
        visionPortal1.close();

        visionPortal2.stopLiveView();
        visionPortal2.stopStreaming();
        visionPortal2.close();

    }

    public boolean setCamSettings(){
        if(visionPortal1.getCameraState() == VisionPortal.CameraState.STREAMING){
            ExposureControl exposureControl1 = visionPortal2.getCameraControl(ExposureControl.class);
            if (exposureControl1.getMode() != ExposureControl.Mode.Manual) {
                exposureControl1.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl1.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
        }else{
            return false;
        }

        if(visionPortal2.getCameraState() == VisionPortal.CameraState.STREAMING){
            ExposureControl exposureControl2 = visionPortal2.getCameraControl(ExposureControl.class);
            if (exposureControl2.getMode() != ExposureControl.Mode.Manual) {
                exposureControl2.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl2.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);

            FocusControl focusControl2 = visionPortal2.getCameraControl(FocusControl.class);
            if(focusControl2.getMode() == FocusControl.Mode.Fixed){
                focusControl2.setFocusLength(focusLength);
            }
        }else{
            return false;
        }
        return true;
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

    public int identifyTeamPropPose(){
        int detection = 0;

        List<Recognition> currentRecognitions = teamPropTFOD.getRecognitions();

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            if (recognition.getLabel() == "Blue Cube") {
                if (x <= 440) {
                    detection = 1;
                } else if (x > 440 && x < 1280) {
                    detection = 2;
                } else if (x >= 1280) {
                    detection = 3;
                }
            }
        }

        return detection;

    }


    public List<Double> AlignToBackdropTag(int desiredBlueTagID, int desiredRedTagID){

        List<AprilTagDetection> currentDetections = tagProcessor1.getDetections();
        for (AprilTagDetection detection : currentDetections) {

            if (detection.metadata != null) {

                if (detection.id == desiredBlueTagID || detection.id == desiredRedTagID) {
                    targetFound = true;
                    desiredTag = detection;
                    break;
                }
            }
        }
        if (targetFound) {

            // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
            double  rangeError      = (desiredTag.ftcPose.y - DESIRED_DISTANCE);
            double  headingError    = desiredTag.ftcPose.yaw;
            double  yawError        = desiredTag.ftcPose.x;

            // Use the speed and turn "gains" to calculate how we want the robot to move.
            drive  = -Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
            strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

        }


        // Apply desired axes motions to the drivetrain.
        List<Double> motorVels = moveRobot(drive, strafe, turn);
        return motorVels;

    }

    private List<Double> moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        List<Double> motorVels = new ArrayList<>();
        motorVels.add(leftFrontPower);
        motorVels.add(rightFrontPower);
        motorVels.add(leftBackPower);
        motorVels.add(rightBackPower);

        return motorVels;
    }

    private void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
