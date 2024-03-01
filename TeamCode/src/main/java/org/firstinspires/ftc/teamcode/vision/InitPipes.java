package org.firstinspires.ftc.teamcode.vision;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.FocusControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
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

    public static double DESIRED_DISTANCE = 10.0;
    static double SPEED_GAIN  =  -0.02  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    static double STRAFE_GAIN =  0.03 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    static double TURN_GAIN   =  -0.01  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    static double MAX_AUTO_SPEED  = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    static double MAX_AUTO_STRAFE = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    static double MAX_AUTO_TURN   = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    double drive = 0;    // Desired forward power/speed (-1 to +1)
    double strafe = 0;   // Desired strafe power/speed (-1 to +1)
    double turn = 0;     // Desired turning power/speed (-1 to +1)

    public static double exposureMS = 5;
    public static double focusLength = 0;

    public static int gain = 255;

    double  rangeError;
    double  headingError;
    double  yawError;

    AprilTagProcessor tagProcessor1;
    AprilTagProcessor tagProcessor2;
    VisionPortal visionPortal1;
    VisionPortal visionPortal2;
    TfodProcessor teamPropTFOD;

    private static final String TFOD_MODEL_ASSET = "HorseTFOD25-02-2024.tflite";
    private static final String[] LABELS = {
            "Blue Horse", "Red Horse"
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
        /**
         * C920 camera 2
         * Focals (pixels) - Fx: 615.421 Fy: 615.421
         * Optical center - Cx: 645.509 Cy: 353.159
         *
         * C270 camera 1
         * Focals (pixels) - Fx: 959.175 Fy: 959.175
         * Optical center - Cx: 406.893 Cy: 234.326
         *
         *
         * ================ OLD NAO USAR! ==================
         * C920
         * Focals (pixels) - Fx: 968.405 Fy: 968.405
         * Optical center - Cx: 604.808 Cy: 364.632
         *
         * C270
         * Focals (pixels) - Fx: 920.729 Fy: 920.729
         * Optical center - Cx: 511.404 Cy: 152.429
         */
        int[] portalsList = VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL);

        tagProcessor1 = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setLensIntrinsics(1308.0568774459832, 1309.0599918491519, 465.6182547784557, 129.1917324755751)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagCustomDatabase.getCenterStageLibrary())
                .build();

        tagProcessor2 = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setLensIntrinsics(1028.4636677848612, 1024.960787540627, 574.8966906772454, 382.2051828835763)
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
                //Supported resolutions are
                // [640x480], [160x120], [176x144], [320x176], [320x240],
                // [352x288], [432x240], [544x288], [640x360], [752x416],
                // [800x448], [800x600], [864x480], [960x544], [960x720],
                // [1024x576], [1184x656], [1280x720], [1280x960],
                .setCameraResolution(new Size(800, 448))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .setLiveViewContainerId(portalsList[0])
                .setAutoStopLiveView(true)
                .build();

        visionPortal2 = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"))
                .addProcessors(tagProcessor2, teamPropTFOD)
                .setCameraResolution(new Size(1280, 720))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .setLiveViewContainerId(portalsList[1])
                .setAutoStopLiveView(true)
                .build();


        visionPortal1.setProcessorEnabled(tagProcessor1, false);
        visionPortal2.setProcessorEnabled(tagProcessor2, false);
        visionPortal2.setProcessorEnabled(teamPropTFOD, false);

        tagProcessor1.setPoseSolver(AprilTagProcessor.PoseSolver.OPENCV_SOLVEPNP_EPNP);
        tagProcessor2.setPoseSolver(AprilTagProcessor.PoseSolver.OPENCV_SOLVEPNP_EPNP);

        while(visionPortal1.getCameraState() != VisionPortal.CameraState.STREAMING);
        while(visionPortal2.getCameraState() != VisionPortal.CameraState.STREAMING);

        //setCamSettings();
    }

    public ArrayList<AprilTagDetection> updateTagProcessor1(){
        return tagProcessor1.getDetections();
    }
    public ArrayList<AprilTagDetection> updateTagProcessor2(){
        return tagProcessor2.getDetections();
    }

    public void closeCams(){
        visionPortal1.close();
        visionPortal2.close();

    }

    public boolean setCamSettings(){
        if(visionPortal1.getCameraState() == VisionPortal.CameraState.STREAMING){
            ExposureControl exposureControl1 = visionPortal1.getCameraControl(ExposureControl.class);
            if (exposureControl1.getMode() != ExposureControl.Mode.Manual) {
                exposureControl1.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl1.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);

            GainControl gaincontrol1  = visionPortal1.getCameraControl(GainControl.class);
            gaincontrol1.setGain(gain);
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

            GainControl gaincontrol2  = visionPortal2.getCameraControl(GainControl.class);
            gaincontrol2.setGain(gain);
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

    // 0 = red
    // 1 = blue
    public int identifyTeamPropPose(int type){
        int detection = 0;

        List<Recognition> currentRecognitions = teamPropTFOD.getRecognitions();

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            if(type == 0){
                if (recognition.getLabel() == "Red Horse") {
                    if (x <= 350) {
                        detection = 1;
                    } else if (x > 350 && x < 880) {
                        detection = 2;
                    } else if (x >= 880) {
                        detection = 3;
                    }
                }
            }
            if (type == 1){
                if (recognition.getLabel() == "Blue Horse") {
                    if (x <= 350) {
                        detection = 1;
                    } else if (x > 350 && x < 880) {
                        detection = 2;
                    } else if (x >= 880) {
                        detection = 3;
                    }
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
            rangeError      = (desiredTag.ftcPose.y - DESIRED_DISTANCE);
            headingError    = desiredTag.ftcPose.yaw;
            yawError        = desiredTag.ftcPose.x;

            // Use the speed and turn "gains" to calculate how we want the robot to move.
            drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            turn   = 0;//-Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
            strafe = -Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

        }


        // Apply desired axes motions to the drivetrain.
        List<Double> motorVels = moveRobot(drive, strafe, turn);
        return motorVels;

    }

    public double returnRangeError(){
        return rangeError;
    }
    public double returnHeadingError(){
        return headingError;
    }
    public double returnYawError(){
        return yawError;
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
