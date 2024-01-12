package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RoadRunnerScripts.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunnerScripts.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.DetectorHSVEDGE;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class PixelRed extends LinearOpMode {

    OpenCvCamera camera;
    private DcMotorEx Intake = null;
    private DcMotorEx Lift = null;
    private Servo garra = null;
    private ServoImplEx cotovelo = null;
    private ServoImplEx ombroR = null;
    private ServoImplEx ombroL = null;

    @Override
    public void runOpMode()  {

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

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        DetectorHSVEDGE colorfilter = new DetectorHSVEDGE();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.setPipeline(colorfilter);
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

        Pose2d startPose = new Pose2d(-36,-63.55, Math.toRadians(90));

        //start trajectory
        TrajectorySequence toSpikeMarks = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-36,-36, Math.toRadians(90)))
                .build();

        TrajectorySequence toBackdropANALISE3 = drive.trajectorySequenceBuilder(new Pose2d(-36,-36, Math.toRadians(0)))
                .turn(Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(-36, -12, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(36, -12, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d( 36, -36, Math.toRadians(180)))
                .build();
        TrajectorySequence toBackdropANALISE1 = drive.trajectorySequenceBuilder(new Pose2d(-36,-36, Math.toRadians(180)))
                .turn(Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(-36, -12, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(36, -12, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d( 36, -36, Math.toRadians(180)))
                .build();
        TrajectorySequence toBackdropANALISE2 = drive.trajectorySequenceBuilder(new Pose2d(-36,-36, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-60, -36, Math.toRadians(90)))
                .lineToSplineHeading(new Pose2d(-60, -12, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(36, -12, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d( 36, -36, Math.toRadians(180)))
                .build();
        TrajectorySequence toLeftAprilTag = drive.trajectorySequenceBuilder(toBackdropANALISE1.end())
                .lineToLinearHeading(new Pose2d(40, -30, Math.toRadians(180)))
                .build();
        TrajectorySequence toMiddleAprilTag = drive.trajectorySequenceBuilder(toBackdropANALISE2.end())
                .lineToLinearHeading(new Pose2d(40, -36, Math.toRadians(180)))
                .build();
        TrajectorySequence toRightAprilTag = drive.trajectorySequenceBuilder(toBackdropANALISE1.end())
                .lineToLinearHeading(new Pose2d(40, -42, Math.toRadians(180)))
                .build();
        TrajectorySequence forward = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(4.5)
                .build();
        TrajectorySequence backward = drive.trajectorySequenceBuilder(new Pose2d())
                .back(4.5)
                .build();
        TrajectorySequence park = drive.trajectorySequenceBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(36, -60, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(60, -60, Math.toRadians(180)))
                .build();

        waitForStart();
        resetRuntime();

        int analysis = 0;
        while(analysis == 0 && isStarted() && getRuntime() < 2){
            analysis = colorfilter.getAnalysis();
        }

        telemetry.addData("Analise: ", analysis);
        telemetry.update();


        drive.followTrajectorySequence(toSpikeMarks);
        sleep(500);

        if (analysis == 1) {
            drive.turn(Math.toRadians(90));
            sleep(200);
            CuspirPixel();
            drive.turn(Math.toRadians(-90));
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

}

