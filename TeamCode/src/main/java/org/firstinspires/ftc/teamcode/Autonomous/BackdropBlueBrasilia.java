package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunnerScripts.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.RoadRunnerScripts.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunnerScripts.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Subsystems.ArmSystem;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSystem;
import org.firstinspires.ftc.teamcode.vision.InitPipes;

@Autonomous
public class BackdropBlueBrasilia extends LinearOpMode {

    IntakeSystem intake;
    ArmSystem arm;
    InitPipes instancia = InitPipes.getInstancia();
    SampleMecanumDrive drive;

    int analysis = 0;

    @Override
    public void runOpMode()  {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //HardwareMap Config
        intake = new IntakeSystem(hardwareMap);
        arm = new ArmSystem(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);

        arm.closeGarra();
        instancia.initVision(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Pose2d startPose = new Pose2d(12,63.55, Math.toRadians(270));
        drive.setPoseEstimate(startPose);

        ///Trajectory Construct
        TrajectorySequence toSpikeMarks = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(12,33.5, Math.toRadians(270)))
                .build();
        TrajectorySequence parkANALISE1 = drive.trajectorySequenceBuilder( new Pose2d(12,33.5, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(12, 12, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(36, 12, Math.toRadians(180)))
                .addSpatialMarker(new Vector2d(36, 24), () -> {arm.UpArm();})
                .lineToLinearHeading(new Pose2d(36, 36, Math.toRadians(180)))
                .build();
        TrajectorySequence parkANALISE2 = drive.trajectorySequenceBuilder(new Pose2d(12,33.5, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(36, 36, Math.toRadians(180)))
                .addSpatialMarker(new Vector2d(24, 34.7), () -> {arm.UpArm();})
                .build();
        TrajectorySequence parkANALISE3 = drive.trajectorySequenceBuilder(new Pose2d(12, 33.5, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(36, 36, Math.toRadians(180)))
                .addSpatialMarker(new Vector2d(24, 34.7), () -> {arm.UpArm();})
                .build();

        TrajectorySequence toLeftAprilTag = drive.trajectorySequenceBuilder(parkANALISE1.end())
                .lineToLinearHeading(new Pose2d(40, 42, Math.toRadians(180)))
                .back(9)
                .addTemporalMarker(() -> {arm.openGarra();})
                .waitSeconds(0.25)
                .forward(9,
                        SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .lineToLinearHeading(new Pose2d(36, 36, Math.toRadians(180)))
                .addTemporalMarker(() -> {arm.DownArm();})
                .waitSeconds(0.1)
                .build();
        TrajectorySequence toMiddleAprilTag = drive.trajectorySequenceBuilder(parkANALISE2.end())
                .lineToLinearHeading(new Pose2d(40, 38, Math.toRadians(180)))
                .back(10)
                .addTemporalMarker(() -> {arm.openGarra();})
                .waitSeconds(0.25)
                .forward(10,
                        SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .lineToLinearHeading(new Pose2d(36, 36, Math.toRadians(180)))
                .addTemporalMarker(() -> {arm.DownArm();})
                .waitSeconds(0.25)
                .build();
        TrajectorySequence toRightAprilTag = drive.trajectorySequenceBuilder(parkANALISE3.end())
                .lineToLinearHeading(new Pose2d(40, 30, Math.toRadians(180)))
                .back(9)
                .addTemporalMarker(() -> {arm.openGarra();})
                .waitSeconds(0.25)
                .forward(9,
                        SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .lineToLinearHeading(new Pose2d(36, 36, Math.toRadians(180)))
                .addTemporalMarker(() -> {arm.DownArm();})
                .waitSeconds(0.25)
                .build();

        TrajectorySequence park = drive.trajectorySequenceBuilder(new Pose2d(36,36, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(36, 60, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(60, 60, Math.toRadians(180)))
                .build();

        while(analysis == 0 && !isStarted()){
            analysis = instancia.identifyTeamPropPose(1);
        }

        waitForStart();
        resetRuntime();

        telemetry.addData("Analise: ", analysis);
        telemetry.update();

        instancia.activateTFODProcessor(false);

        drive.followTrajectorySequence(toSpikeMarks);
        sleep(500);

        if (analysis == 1) {
            drive.turn(Math.toRadians(90));
            sleep(200);
            intake.cuspirPixel();
            sleep(200);
            drive.followTrajectorySequence(parkANALISE1);
            sleep(200);
            drive.followTrajectorySequence(toLeftAprilTag);
        } else if (analysis == 3){
            drive.turn(Math.toRadians(-90));
            sleep(200);
            intake.cuspirPixel();
            sleep(200);
            drive.followTrajectorySequence(parkANALISE3);
            sleep(200);
            drive.followTrajectorySequence(toRightAprilTag);
        } else {
            intake.cuspirPixel();
            sleep(200);
            drive.followTrajectorySequence(parkANALISE2);
            sleep(200);
            drive.followTrajectorySequence(toMiddleAprilTag);

        }

        drive.followTrajectorySequence(park);

    }

}

