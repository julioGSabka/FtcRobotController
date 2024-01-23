package org.firstinspires.ftc.teamcode.Autonomous;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.RoadRunnerScripts.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunnerScripts.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.AprilTagCustomDatabase;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Autonomous
public class BackdropRed extends LinearOpMode {

    AprilTagProcessor tagProcessor1;
    AprilTagProcessor tagProcessor2;
    VisionPortal visionPortal1;
    VisionPortal visionPortal2;

    private static final String TFOD_MODEL_ASSET = "model_20240115_155137.tflite";
    private static final String[] LABELS = {
            "Blue Cube", "Red Cube"
    };
    private TfodProcessor tfod;

    private DcMotorEx Intake = null;
    private DcMotorEx Lift = null;
    private Servo garra = null;
    private ServoImplEx cotovelo = null;
    private ServoImplEx ombroR = null;
    private ServoImplEx ombroL = null;

    @Override
    public void runOpMode()  {

        //HardwareMap Config
        //Motors
        Intake = hardwareMap.get(DcMotorEx.class,"Intake"); //Ex0
        Lift = hardwareMap.get(DcMotorEx.class, "Lift"); //Ex1
        //Servos
        garra = hardwareMap.get(Servo.class, "garra"); //Ex0
        cotovelo = hardwareMap.get(ServoImplEx.class, "cotovelo"); //4
        ombroR = hardwareMap.get(ServoImplEx.class, "ombroR"); //2
        ombroL = hardwareMap.get(ServoImplEx.class, "ombroL"); //0

        Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Lift.setDirection(DcMotorSimple.Direction.REVERSE);

        //System Adjustments
        garra.setPosition(0);
        cotovelo.setPosition(0.6);
        ombroL.setPosition(0);
        ombroR.setPosition(1);
        sleep(1000);
        cotovelo.setPosition(0.5);
        sleep(1000);
        DisableServos();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(12,-63.55, Math.toRadians(90));

        //Trajectory Construct
        TrajectorySequence toSpikeMarks = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(12,-36, Math.toRadians(90)))
                .build();
        TrajectorySequence toBackdropMiddle = drive.trajectorySequenceBuilder(new Pose2d(12,-36, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(12,-12, Math.toRadians(0)))
                .lineToSplineHeading(new Pose2d(36,-12, Math.toRadians(180)))
                .strafeTo(new Vector2d(36, -36))
                .build();
        TrajectorySequence toBackdropRightAnalysis1 = drive.trajectorySequenceBuilder(new Pose2d(12,-36, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(36, -36, Math.toRadians(180)))
                .build();
        TrajectorySequence toBackdropRightAnalysis2 = drive.trajectorySequenceBuilder(new Pose2d(12,-36, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(36, -36, Math.toRadians(180)))
                .build();
        TrajectorySequence toLeftAprilTag = drive.trajectorySequenceBuilder(new Pose2d(12,-36, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(40, -30, Math.toRadians(180)))
                .build();
        TrajectorySequence toMiddleAprilTag = drive.trajectorySequenceBuilder(new Pose2d(12,-36, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(40, -36, Math.toRadians(180)))
                .build();
        TrajectorySequence toRightAprilTag = drive.trajectorySequenceBuilder(new Pose2d(12,-36, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(40, -42, Math.toRadians(180)))
                .build();
        TrajectorySequence forward = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(4.5)
                .build();
        TrajectorySequence backward = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(-4.5)
                .build();
        TrajectorySequence park = drive.trajectorySequenceBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(36, -60, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(60, -60, Math.toRadians(180)))
                .build();

        initVisionPipelines(); //Inicialização da Visão

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        resetRuntime();

        int analysis = 0;
        while(analysis == 0 && isStarted() && getRuntime() < 3.5){
            analysis = detectTfod();
        }

        telemetry.addData("Analise: ", analysis);
        telemetry.update();

        drive.followTrajectorySequence(toSpikeMarks);
        sleep(500);

        if (analysis == 1) {
            drive.turn(Math.toRadians(90));
            sleep(200);
            CuspirPixel();

            drive.followTrajectorySequence(toBackdropRightAnalysis1);
            sleep(500);
            drive.followTrajectorySequence(toLeftAprilTag);
        } else if (analysis == 2) {
            CuspirPixel();

            drive.followTrajectorySequence(toBackdropRightAnalysis2);
            sleep(500);
            drive.followTrajectorySequence(toMiddleAprilTag);
        } else if (analysis == 3) {
            drive.turn(Math.toRadians(-90));
            sleep(200);
            CuspirPixel();

            drive.followTrajectorySequence(toBackdropMiddle);
            sleep(500);
            drive.followTrajectorySequence(toRightAprilTag);
        }

        LevantarBraco();
        drive.followTrajectorySequence(forward);
        sleep(200);
        garra.setPosition(0);
        sleep(200);
        drive.followTrajectorySequence(backward);
        BaixarBraco();
        sleep(200);
        drive.followTrajectorySequence(park);

    }

    public void DisableServos(){
        cotovelo.setPwmDisable();
        ombroR.setPwmDisable();
        ombroL.setPwmDisable();
    }

    public void EnableServos(){
        cotovelo.setPwmEnable();
        ombroR.setPwmEnable();
        ombroL.setPwmEnable();
    }

    public void LevantarBraco() {
        EnableServos();
        cotovelo.setPosition(1);
        sleep(1000);
        ombroL.setPosition(1);
        ombroR.setPosition(0);
        sleep(500);
        cotovelo.setPosition(0.1);
    }

    public void BaixarBraco() {
        EnableServos();
        cotovelo.setPosition(1);
        sleep(500);
        ombroL.setPosition(0);
        ombroR.setPosition(1);
        sleep(1000);
        cotovelo.setPosition(0.4);
        sleep(1000);
        DisableServos();
    }

    public void CuspirPixel(){
        Intake.setPower(2);
        sleep(1000);
        Intake.setPower(0);
        sleep(200);
    }


    private void initVisionPipelines() {

        int[] portalsList = VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL);

        tfod = new TfodProcessor.Builder()
                .setModelAssetName(TFOD_MODEL_ASSET)
                .setModelLabels(LABELS)
                .build();

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
                .addProcessors(tfod, tagProcessor2)
                .setCameraResolution(new Size(1920, 1080))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .setLiveViewContainerId(portalsList[1])
                .setAutoStopLiveView(true)
                .build();

        visionPortal1.setProcessorEnabled(tagProcessor1, true);
        visionPortal2.setProcessorEnabled(tagProcessor2, true);
    }

    private int detectTfod() {
        int detection = 0;

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("Objects Detected", currentRecognitions.size());
        telemetry.update();

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            if (recognition.getLabel() == "Red Cube") {
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

}
