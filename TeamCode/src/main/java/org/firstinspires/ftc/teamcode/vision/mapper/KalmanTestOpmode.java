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
import org.firstinspires.ftc.teamcode.vision.InitPipes;
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

    List<Pose2d> measurePoses;
    List<Double> wheelsVels = new ArrayList<>();

    InitPipes instancia = InitPipes.getInstancia();

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

    double lateralWheelDistance = 14.996;
    double verticalWheelDistance = 13.228;

    KalmanPose kalmanPose;

    IMU imu;

    double WHEEL_RADIUS = 1.889;
    double GEAR_RATIO = 1.0;
    double TICKS_PER_REV = 537.7;

    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private List<DcMotorEx> motors;

    private List<Integer> lastEncVels = new ArrayList<>();

    MecanumKinematics mecdrive = new MecanumKinematics();

    @Override
    public void runOpMode() {

        leftFront = hardwareMap.get(DcMotorEx.class, "motorFrontLeft");
        leftRear = hardwareMap.get(DcMotorEx.class, "motorBackLeft");
        rightRear = hardwareMap.get(DcMotorEx.class, "motorBackRight");
        rightFront = hardwareMap.get(DcMotorEx.class, "motorFrontRight");

        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        rightRear.setDirection(DcMotorEx.Direction.REVERSE);

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));

        measurePoses = new ArrayList<>();
        kalmanPose = new KalmanPose(new Pose2d(0, 0, new Rotation2d(0)));

        instancia.initVision(hardwareMap);

        telemetry.addLine("Waiting for start");
        telemetry.update();

        dashboard = FtcDashboard.getInstance();

        waitForStart();

        imu.resetYaw();

        tagProcessor1 = instancia.returnTagProcessor1();
        tagProcessor2 = instancia.returnTagProcessor2();
        while (opModeIsActive()) {
            tagTelemetry();
        }

        instancia.closeCams();
    }

    public void tagTelemetry(){
        measurePoses.clear();
        TelemetryPacket packet = new TelemetryPacket();
        if (tagProcessor1.getDetections().size() > 0){
            ArrayList<AprilTagDetection> tags = tagProcessor1.getDetections();
            for(AprilTagDetection tag : tags){

                if (tag.decisionMargin > 120){
                    Pose2d rpose = Positioner.getRobotPose(tag, new Transform2d(new Translation2d(-8, 0), new Rotation2d(Math.toRadians(180))));
                    measurePoses.add(rpose);

                    packet.fieldOverlay()
                            .setStroke(colors[tag.id])
                            .strokeCircle(rpose.getX(), rpose.getY(), 10)
                            .strokeLine(rpose.getX(), rpose.getY(), rpose.getX() + 10*rpose.getRotation().getCos(), rpose.getY()+ 10*rpose.getRotation().getSin());
                }

            }
        }

        if (tagProcessor2.getDetections().size() > 0){

            ArrayList<AprilTagDetection> tags = tagProcessor2.getDetections();

            for (AprilTagDetection tag : tags){
                if (tag.decisionMargin > 120){
                    Pose2d rpose = Positioner.getRobotPose(tag, new Transform2d(new Translation2d(8, 0), new Rotation2d(Math.toRadians(0))));
                    measurePoses.add(rpose);

                    packet.fieldOverlay()
                            .setStroke(colors[tag.id])
                            .strokeCircle(rpose.getX(), rpose.getY(), 10)
                            .strokeLine(rpose.getX(), rpose.getY(), rpose.getX() + 10*rpose.getRotation().getCos(), rpose.getY()+ 10*rpose.getRotation().getSin());

                }
            }
        }

        //obtençao do ângulo
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.RADIANS);

        //Chama o fitro e passa os valores (vel, statePose, measurePoses)

        Pose2d delPose = mecdrive.mecanumDeltaPose(
                encoderTicksToInches(leftFront.getCurrentPosition()),
                encoderTicksToInches(rightFront.getCurrentPosition()),
                encoderTicksToInches(leftRear.getCurrentPosition()),
                encoderTicksToInches(rightRear.getCurrentPosition()),
                verticalWheelDistance,
                lateralWheelDistance
                );
        packet.put("DP", delPose);

        if(measurePoses.size() > 0){
            //Obtenção da velocidade do robo(vel)
            /*
            List<Double> TickVels = getWheelVelocities();
            for (double vels : TickVels){
                wheelsVels.add(encoderTicksToInches(vels));
            }
             */
            /*
            Pose2d vel = MecanumKinematics.wheelToVel(
                    encoderTicksToInches(leftFront.getVelocity()),
                    encoderTicksToInches(rightFront.getVelocity()),
                    encoderTicksToInches(leftRear.getVelocity()),
                    encoderTicksToInches(rightRear.getVelocity()),
                    15.15749,
                    13.3859);
             */
            //Drive: FL, Bl, BR, FR
            //wheelToVel: FL, FR, BL, BR
            //packet.put("mecanumVel", vel);
            kalmanPose.updateFilter(delPose, measurePoses, orientation.getYaw(AngleUnit.RADIANS));
        }else{
            /*
            Pose2d delPose = mecdrive.mecanumDeltaPose(
                    encoderTicksToInches(leftFront.getCurrentPosition()),
                    encoderTicksToInches(rightFront.getCurrentPosition()),
                    encoderTicksToInches(leftRear.getCurrentPosition()),
                    encoderTicksToInches(rightRear.getCurrentPosition()),
                    15.15749,
                    13.3859
                    );

             */
            double orient = orientation.getYaw(AngleUnit.RADIANS);
            kalmanPose.addDelta(delPose.getTranslation().rotateBy(new Rotation2d(orient)), orient);
        }
        Pose2d filteredPose = kalmanPose.getPose();

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
