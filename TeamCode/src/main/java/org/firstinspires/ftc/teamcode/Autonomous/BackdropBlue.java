package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.RoadRunnerScripts.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunnerScripts.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;

@Autonomous
public class BackdropBlue extends LinearOpMode {
    OpenCvCamera camera;
    private DcMotorEx Intake = null;
    private DcMotorEx Lift = null;
    private Servo garra = null;
    private ServoImplEx cotovelo = null;
    private ServoImplEx ombroR = null;
    private ServoImplEx ombroL = null;

    private static final boolean USE_WEBCAM = true;
    private static final String TFOD_MODEL_ASSET = "model_20240115_155137.tflite";
    private static final String[] LABELS = {
            "Blue Cube", "Red Cube"
    };
    private TfodProcessor tfod;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode()  {
        initTfod();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

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

        //initTags();
        //visionPortal1.stopStreaming();
        //visionPortal2.stopStreaming();

        garra.setPosition(0);
        cotovelo.setPosition(0.6);
        ombroL.setPosition(0);
        ombroR.setPosition(1);
        sleep(1000);
        cotovelo.setPosition(0.5);
        sleep(1000);
        DisableServos();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(12,63.55, Math.toRadians(270));

        //start trajectory
        TrajectorySequence toSpikeMarks = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(12,36, Math.toRadians(270)))
                .build();
        TrajectorySequence toBackdropANALISE1 = drive.trajectorySequenceBuilder(new Pose2d(12,36, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(12,12, Math.toRadians(270)))
                .lineToSplineHeading(new Pose2d(36,12, Math.toRadians(180)))
                .strafeTo(new Vector2d(36, 36))
                .build();
        TrajectorySequence toBackdropANALISE3 = drive.trajectorySequenceBuilder(new Pose2d(12,36, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(36, 36, Math.toRadians(180)))
                .build();
        TrajectorySequence toBackdropANALISE2 = drive.trajectorySequenceBuilder(new Pose2d(12,36, Math.toRadians(270)))
                .turn(Math.toRadians(-90))
                .lineToLinearHeading(new Pose2d(36, 36, Math.toRadians(180)))
                .build();
        TrajectorySequence toLeftAprilTag = drive.trajectorySequenceBuilder(new Pose2d(12,36, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(40, 42, Math.toRadians(180)))
                .build();
        TrajectorySequence toMiddleAprilTag = drive.trajectorySequenceBuilder(new Pose2d(12,36, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(40, 36, Math.toRadians(180)))
                .build();
        TrajectorySequence toRightAprilTag = drive.trajectorySequenceBuilder(new Pose2d(12,36, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(40, 30, Math.toRadians(180)))
                .build();
        TrajectorySequence forward = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(4.5)
                .build();
        TrajectorySequence backward = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(-4.5)
                .build();
        TrajectorySequence park = drive.trajectorySequenceBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(36, 60, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(60, 60, Math.toRadians(180)))
                .build();

        waitForStart();
        resetRuntime();

        drive.followTrajectorySequence(toSpikeMarks);
        sleep(500);

        int analysis = 0;
        while (analysis == 0) {
            detectTfod();
        }

        if (analysis == 1) {
            drive.turn(Math.toRadians(90));
            sleep(200);
            CuspirPixel();
            drive.followTrajectorySequence(toBackdropANALISE1);
            sleep(500);
            drive.followTrajectorySequence(toLeftAprilTag);
        } else if (analysis == 2) {
            CuspirPixel();
            drive.followTrajectorySequence(toBackdropANALISE2);
            sleep(500);
            drive.followTrajectorySequence(toMiddleAprilTag);
        } else if (analysis == 3) {
            drive.turn(Math.toRadians(-90));
            sleep(200);
            CuspirPixel();

            drive.followTrajectorySequence(toBackdropANALISE3);
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
        Intake.setPower(-2);
        sleep(1000);
        Intake.setPower(0);
        sleep(200);
    }

    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder().setModelAssetName(TFOD_MODEL_ASSET).setModelLabels(LABELS).build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();
    }

    private void detectTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop


    }
}

