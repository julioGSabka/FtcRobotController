package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.RoadRunnerScripts.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunnerScripts.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSystem;
import org.firstinspires.ftc.teamcode.vision.InitPipes;

import java.util.List;

@Autonomous
public class BackdropRedSemBackdrop extends LinearOpMode {

    IntakeSystem intake;
    InitPipes instancia = InitPipes.getInstancia();

    int analysis = 0;

    @Override
    public void runOpMode()  {

        //HardwareMap Config
        intake = new IntakeSystem(hardwareMap);

        instancia.initVision(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(12,-63.55, Math.toRadians(90));

        //Trajectory Construct
        TrajectorySequence toSpikeMarks = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(12,-36, Math.toRadians(90)))
                .build();
        TrajectorySequence parkANALISE1 = drive.trajectorySequenceBuilder( new Pose2d(12,-36, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(48, -36, Math.toRadians(180)))
                .build();
        TrajectorySequence parkANALISE2 = drive.trajectorySequenceBuilder(new Pose2d(12,-36, Math.toRadians(180)))
                .back(10)
                .lineToLinearHeading(new Pose2d(48, -36, Math.toRadians(180)))
                .build();
        TrajectorySequence parkANALISE3 = drive.trajectorySequenceBuilder(new Pose2d(12, -36, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(12, -12, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(55, -12, Math.toRadians(0)))
                .build();

        while(analysis == 0 && !isStarted()){
            analysis = detectTfod();
        }

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        resetRuntime();

        while(analysis == 0 && isStarted() && getRuntime() < 3.5){
            analysis = detectTfod();
        }

        telemetry.addData("Analise: ", analysis);
        telemetry.update();

        instancia.activateTFODProcessor(false);

        drive.followTrajectorySequence(toSpikeMarks);
        sleep(500);

        if (analysis == 1) {
            drive.turn(Math.toRadians(90));
            sleep(200);
            intake.cuspirPixel();
            sleep(1500);
            drive.followTrajectorySequence(parkANALISE1);

        } else if (analysis == 3) {
            drive.turn(Math.toRadians(-90));
            sleep(200);
            intake.cuspirPixel();
            sleep(750);
            drive.followTrajectorySequence(parkANALISE3);

        } else {
            intake.cuspirPixel();
            sleep(1500);
            drive.turn(Math.toRadians(90));
            drive.followTrajectorySequence(parkANALISE2);

        }

    }

    private int detectTfod() {
        int detection = 0;

        List<Recognition> currentRecognitions = instancia.returnTFOD().getRecognitions();
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