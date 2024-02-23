package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunnerScripts.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunnerScripts.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Subsystems.ArmSystem;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSystem;
import org.firstinspires.ftc.teamcode.vision.InitPipes;

@Autonomous
public class PixelBlue extends LinearOpMode {

    ArmSystem arm;
    IntakeSystem intake;
    InitPipes instancia = InitPipes.getInstancia();

    int analysis = 0;

    @Override
    public void runOpMode()  {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //HardwareMap Config
        arm = new ArmSystem(hardwareMap);
        intake = new IntakeSystem(hardwareMap);

        instancia.initVision(hardwareMap);

        //System Adjustments
        arm.closeGarra();
        arm.DownArm();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-36,63.55, Math.toRadians(270));
        drive.setPoseEstimate(startPose);

        //start trajectory
        TrajectorySequence toSpikeMarks = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-36,36, Math.toRadians(270)))
                .build();

        TrajectorySequence toBackdropANALISE1 = drive.trajectorySequenceBuilder(new Pose2d(-36,36, Math.toRadians(0)))
                .turn(Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(-36, 12, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(36, 12, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d( 36, 36, Math.toRadians(180)))
                .build();
        TrajectorySequence toBackdropANALISE3 = drive.trajectorySequenceBuilder(new Pose2d(-36,36, Math.toRadians(180)))
                .turn(Math.toRadians(-90))
                .lineToLinearHeading(new Pose2d(-36, 12, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(36, 12, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d( 36, 36, Math.toRadians(180)))
                .build();
        TrajectorySequence toBackdropANALISE2 = drive.trajectorySequenceBuilder(new Pose2d(-36,36, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(-60, 36, Math.toRadians(270)))
                .lineToSplineHeading(new Pose2d(-60, 12, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(36, 12, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d( 36, 36, Math.toRadians(180)))
                .build();
        TrajectorySequence toLeftAprilTag = drive.trajectorySequenceBuilder(toBackdropANALISE1.end())
                .lineToLinearHeading(new Pose2d(40, 42, Math.toRadians(180)))
                .build();
        TrajectorySequence toMiddleAprilTag = drive.trajectorySequenceBuilder(toBackdropANALISE2.end())
                .lineToLinearHeading(new Pose2d(40, 36, Math.toRadians(180)))
                .build();
        TrajectorySequence toRightAprilTag = drive.trajectorySequenceBuilder(toBackdropANALISE3.end())
                .lineToLinearHeading(new Pose2d(40, 30, Math.toRadians(180)))
                .build();
        TrajectorySequence forward = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .forward(4.5)
                .build();
        TrajectorySequence backward = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .back(4.5)
                .build();
        TrajectorySequence park = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(36, 60, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(60, 60, Math.toRadians(180)))
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

        drive.followTrajectorySequence(toSpikeMarks);
        sleep(500);

        if (analysis == 1) {
            drive.turn(Math.toRadians(90));
            sleep(200);
            intake.cuspirPixel();
            drive.followTrajectorySequence(toBackdropANALISE1);
            sleep(500);
            drive.followTrajectorySequence(toRightAprilTag);
        } else if (analysis == 2) {
            intake.cuspirPixel();
            drive.followTrajectorySequence(toBackdropANALISE2);
            sleep(500);
            drive.followTrajectorySequence(toMiddleAprilTag);
        } else if (analysis == 3) {
            drive.turn(Math.toRadians(-90));
            sleep(200);
            intake.cuspirPixel();
            drive.followTrajectorySequence(toBackdropANALISE3);
            sleep(500);
            drive.followTrajectorySequence(toLeftAprilTag);
        }

        arm.UpArm();
        drive.followTrajectorySequence(forward);
        sleep(200);
        arm.openGarra();
        sleep(200);
        drive.followTrajectorySequence(backward);
        arm.DownArm();
        sleep(200);
        drive.followTrajectorySequence(park);

    }

}
