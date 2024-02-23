package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunnerScripts.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunnerScripts.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSystem;
import org.firstinspires.ftc.teamcode.vision.InitPipes;

import java.util.List;

@Autonomous
public class PixelRedSemBackdrop extends LinearOpMode {

    IntakeSystem intake;
    InitPipes instancia = InitPipes.getInstancia();

    int analysis = 0;

    @Override
    public void runOpMode()  {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //HardwareMap Config
        intake = new IntakeSystem(hardwareMap);

        instancia.initVision(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-36,-63.55, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        //start trajectory
        TrajectorySequence toSpikeMarks = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-36,-31, Math.toRadians(90)))
                .build();
        TrajectorySequence parkANALISE1 = drive.trajectorySequenceBuilder(new Pose2d(-36,-36, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(-36, -12, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(36, -12, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(36, -36, Math.toRadians(180)))
                .build();
        TrajectorySequence parkANALISE2 = drive.trajectorySequenceBuilder(toSpikeMarks.end())
                .lineToLinearHeading(new Pose2d(-60, -36, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-60, -12, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(60, -12, Math.toRadians(90)))
                .build();
        TrajectorySequence parkANALISE3 = drive.trajectorySequenceBuilder(new Pose2d(-36,-36, Math.toRadians(0)))
                .back(5)
                .turn(Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(-36, -12, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(60, -12, Math.toRadians(90)))
                .build();

        while(analysis == 0 && !isStarted()){
            analysis = instancia.identifyTeamPropPose();
        }

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        resetRuntime();

        while(analysis == 0 && isStarted() && getRuntime() < 3.5){
            analysis = instancia.identifyTeamPropPose();
        }

        telemetry.addData("Analise: ", analysis);
        telemetry.update();

        instancia.activateTFODProcessor(false);

        //Start Movement
        drive.followTrajectorySequence(toSpikeMarks);
        sleep(500);

        if (analysis == 2) {
            intake.cuspirPixel();
            sleep(1500);
            drive.followTrajectorySequence(parkANALISE2);

        } else if (analysis == 3){
            drive.turn(Math.toRadians(-90));
            sleep(200);
            intake.cuspirPixel();
            sleep(750);
            drive.followTrajectorySequence(parkANALISE3);
        } else {
            drive.turn(Math.toRadians(90));
            sleep(200);
            intake.cuspirPixel();
            sleep(750);
            drive.followTrajectorySequence(parkANALISE1);

        }


        if (analysis == 1) {
            while (instancia.returnRangeError() > 10 && !isStopRequested()){
                List<Double> motorVels = instancia.AlignToBackdropTag(1, 4);
                drive.setMotorPowers(motorVels.get(0), motorVels.get(1), motorVels.get(2), motorVels.get(3));
            }
        }else if (analysis == 2) {
            while (instancia.returnRangeError() > 10 && !isStopRequested()){
                List<Double> motorVels = instancia.AlignToBackdropTag(2, 5);
                drive.setMotorPowers(motorVels.get(0), motorVels.get(1), motorVels.get(2), motorVels.get(3));
            }
        } else if (analysis == 3) {
            while (instancia.returnRangeError() > 10 && !isStopRequested()){
                List<Double> motorVels = instancia.AlignToBackdropTag(3, 6);
                drive.setMotorPowers(motorVels.get(0), motorVels.get(1), motorVels.get(2), motorVels.get(3));
            }
        }


    }

}