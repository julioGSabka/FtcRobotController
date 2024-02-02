package org.firstinspires.ftc.teamcode.vision.mapper;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.RoadRunnerScripts.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.vision.AprilTagCustomDatabase;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@TeleOp
public class KalmanTestOpmode extends LinearOpMode {

    AprilTagProcessor tagProcessor1;
    AprilTagProcessor tagProcessor2;
    VisionPortal visionPortal1;
    VisionPortal visionPortal2;

    List<Pose2d> measurePoses = new ArrayList<>();;
    List<Double> wheelsVels = new ArrayList<>();;

    FtcDashboard dashboard;
    String[] colors = {
            "red",
            "green",
            "blue",
            "yellow",
            "cyan",
            "magenta",
            "orange",
            "purple",
            "gray",
            "black"
    };
    KalmanPose kalmanPose;

    IMU imu;

    double WHEEL_RADIUS = 1.47;
    double GEAR_RATIO = 1.0;
    double TICKS_PER_REV = 537.6;

    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private List<DcMotorEx> motors;

    private List<Integer> lastEncVels = new ArrayList<>();

    @Override
    public void runOpMode() {

        leftFront = hardwareMap.get(DcMotorEx.class, "motorFrontLeft");
        leftRear = hardwareMap.get(DcMotorEx.class, "motorBackLeft");
        rightRear = hardwareMap.get(DcMotorEx.class, "motorBackRight");
        rightFront = hardwareMap.get(DcMotorEx.class, "motorFrontRight");

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));

        measurePoses = new ArrayList<>();
        kalmanPose = new KalmanPose(new Pose2d(0, 0, new Rotation2d(0)));

        int[] portalsList = VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL);

        tagProcessor1 = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setLensIntrinsics(1219.057, 1220.919, 617.782, 339.444)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagCustomDatabase.getCenterStageLibrary())
                .build();

        tagProcessor2 = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setLensIntrinsics(1494.943, 1490.958, 934.017, 549.413)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagCustomDatabase.getCenterStageLibrary())
                .build();

        visionPortal1 = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(tagProcessor1)
                .setCameraResolution(new Size(1280, 720))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .setLiveViewContainerId(portalsList[0])
                .setAutoStopLiveView(true)
                .build();

        visionPortal2 = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"))
                .addProcessor(tagProcessor2)
                .setCameraResolution(new Size(1920, 1080))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .setLiveViewContainerId(portalsList[1])
                .setAutoStopLiveView(true)
                .build();

        visionPortal1.setProcessorEnabled(tagProcessor1, true);
        visionPortal2.setProcessorEnabled(tagProcessor2, true);

        telemetry.addLine("Waiting for start");
        telemetry.update();

        tagProcessor1.setPoseSolver(AprilTagProcessor.PoseSolver.OPENCV_SOLVEPNP_EPNP);
        tagProcessor2.setPoseSolver(AprilTagProcessor.PoseSolver.OPENCV_SOLVEPNP_EPNP);
        dashboard = FtcDashboard.getInstance();

        waitForStart();

        imu.resetYaw();

        while (opModeIsActive()) {
            tagTelemetry();
        }

        visionPortal1.stopLiveView();
        visionPortal2.stopLiveView();
        visionPortal1.stopStreaming();
        visionPortal2.stopStreaming();
        visionPortal1.close();
        visionPortal2.close();
    }

    public void tagTelemetry(){
        TelemetryPacket packet = new TelemetryPacket();
        if (tagProcessor1.getDetections().size() > 0){
            ArrayList<AprilTagDetection> tags = tagProcessor1.getDetections();
            for(AprilTagDetection tag : tags){

                Pose2d rpose = Positioner.getRobotPose(tag, new Transform2d(new Translation2d(-6.5, 0), new Rotation2d(Math.toRadians(180))));
                measurePoses.add(rpose);

                //Positioner.tagToCamPose(tag);
                packet.put("tag X", tag.ftcPose.x);
                packet.put("tag Y", tag.ftcPose.y);
                packet.put("tag rot DEGREES:", tag.ftcPose.yaw);
                packet.put("tag teoreticalpose", Positioner.tagTheoreticalPose(tag));

                packet.put("pose ", rpose);

                packet.fieldOverlay()
                        .setStroke(colors[tag.id])
                        .strokeCircle(rpose.getX(), rpose.getY(), 10)
                        .strokeLine(rpose.getX(), rpose.getY(), rpose.getX() + 10*rpose.getRotation().getCos(), rpose.getY()+ 10*rpose.getRotation().getSin());
            }
        }

        if (tagProcessor2.getDetections().size() > 0){

            ArrayList<AprilTagDetection> tags = tagProcessor2.getDetections();

            for (AprilTagDetection tag : tags){

                Pose2d rpose = Positioner.getRobotPose(tag, new Transform2d(new Translation2d(-6.5, 0), new Rotation2d(Math.toRadians(0))));
                measurePoses.add(rpose);

                packet.put("tag X", tag.ftcPose.x);
                packet.put("tag Y", tag.ftcPose.y);
                packet.put("tag rot DEGREES:", tag.ftcPose.yaw);
                packet.put("tag teoreticalpose", Positioner.tagTheoreticalPose(tag));

                packet.put("pose ", rpose);

                packet.fieldOverlay()
                        .setStroke(colors[tag.id])
                        .strokeCircle(rpose.getX(), rpose.getY(), 10)
                        .strokeLine(rpose.getX(), rpose.getY(), rpose.getX() + 10*rpose.getRotation().getCos(), rpose.getY()+ 10*rpose.getRotation().getSin());

            }
        }

        //Obtenção da velocidade do robo(vel)
        List<Double> TickVels = getWheelVelocities();
        for (double vels : TickVels){
            wheelsVels.add(encoderTicksToInches(vels));
        }
        Pose2d vel = MecanumKinematics.wheelToVel(wheelsVels.get(0), wheelsVels.get(3), wheelsVels.get(1), wheelsVels.get(2), 15.15749, 13.3859);
        //Drive: FL, Bl, BR, FR
        //wheelToVel: FL, FR, BL, BR
        packet.put("mecanumVel", vel);
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.RADIANS);

        //Chama o fitro e passa os valores (vel, statePose, measurePoses)

        Pose2d filteredPose = kalmanPose.updateFilter(vel, measurePoses, orientation.getYaw(AngleUnit.RADIANS));

        //Desenha a estimativa
        packet.fieldOverlay()
                .setStroke("black")
                .strokeCircle(filteredPose.getX(), filteredPose.getY(), 10)
                .strokeLine(filteredPose.getX(), filteredPose.getY(), filteredPose.getX() + 10*filteredPose.getRotation().getCos(), filteredPose.getY()+ 10*filteredPose.getRotation().getSin());
        packet.put("FilteredPose", filteredPose);

        dashboard.sendTelemetryPacket(packet);
        //telemetry.update();

    }

    public double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public List<Double> getWheelVelocities() {
        lastEncVels.clear();

        List<Double> wheelVelocities = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            int vel = (int) motor.getVelocity();
            lastEncVels.add(vel);
            wheelVelocities.add(encoderTicksToInches(vel));
        }
        return wheelVelocities;
    }

}
