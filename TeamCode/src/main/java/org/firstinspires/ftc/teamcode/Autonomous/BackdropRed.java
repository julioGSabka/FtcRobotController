package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunnerScripts.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunnerScripts.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Subsystems.ArmSystem;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSystem;
import org.firstinspires.ftc.teamcode.vision.InitPipes;

@Autonomous
public class BackdropRed extends LinearOpMode {

    ArmSystem arm;
    IntakeSystem intake;

    InitPipes instancia = InitPipes.getInstancia();

    int analysis = 0;

    @Override
    public void runOpMode()  {

        //HardwareMap Config
        arm = new ArmSystem(hardwareMap);
        intake = new IntakeSystem(hardwareMap);

        instancia.initVision(hardwareMap);

        //System Adjustments
        arm.closeGarra();
        arm.DownArm();

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

            drive.followTrajectorySequence(toBackdropRightAnalysis1);
            sleep(500);
            drive.followTrajectorySequence(toLeftAprilTag);
        } else if (analysis == 2) {
            intake.cuspirPixel();

            drive.followTrajectorySequence(toBackdropRightAnalysis2);
            sleep(500);
            drive.followTrajectorySequence(toMiddleAprilTag);
        } else if (analysis == 3) {
            drive.turn(Math.toRadians(-90));
            sleep(200);
            intake.cuspirPixel();

            drive.followTrajectorySequence(toBackdropMiddle);
            sleep(500);
            drive.followTrajectorySequence(toRightAprilTag);
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
