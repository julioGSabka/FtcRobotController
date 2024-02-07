package org.firstinspires.ftc.teamcode.RoadRunnerScripts.drive;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.vision.InitPipes;
import org.firstinspires.ftc.teamcode.vision.mapper.KalmanPose;
import org.firstinspires.ftc.teamcode.vision.mapper.MecanumKinematics;
import org.firstinspires.ftc.teamcode.vision.mapper.Positioner;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class ApriltagLocalizer implements Localizer {

    List<com.arcrobotics.ftclib.geometry.Pose2d> measurePoses = new ArrayList<>();;
    List<Double> wheelsVels = new ArrayList<>();;

    InitPipes instancia = InitPipes.getInstancia();

    KalmanPose kalmanPose;

    IMU imu;

    double WHEEL_RADIUS = 1.47;
    double GEAR_RATIO = 1.0;
    double TICKS_PER_REV = 537.6;

    double lateralWheelDistance = 14.996;
    double verticalWheelDistance = 13.228;

    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private List<DcMotorEx> motors;

    private List<Integer> lastEncVels = new ArrayList<>();

    Pose2d filterPose;
    com.arcrobotics.ftclib.geometry.Pose2d filteredPose;

    MecanumKinematics mecdrive = new MecanumKinematics();

    public ApriltagLocalizer(HardwareMap hardwareMap) {
        new ApriltagLocalizer(hardwareMap, true);
    }

    public ApriltagLocalizer(HardwareMap hardwareMap, boolean resetPos) {
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
        kalmanPose = new KalmanPose(new com.arcrobotics.ftclib.geometry.Pose2d(0, 0, new Rotation2d(0)));

        instancia.initVision(hardwareMap);
        instancia.activateTFODProcessor(false);

        imu.resetYaw();
    }

    @NonNull
    @Override
    public Pose2d getPoseEstimate() {

        filterPose = new Pose2d(filteredPose.getX(), filteredPose.getY(), filteredPose.getHeading());

        return filterPose;
    }

    @Override
    public void setPoseEstimate(@NonNull Pose2d pose2d) {
        filterPose = pose2d;
    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        return null;
    }

    @Override
    public void update() {
        calculatePos();
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


    public void calculatePos(){
        measurePoses.clear();
        if (instancia.returnTagProcessor1().getDetections().size() > 0){
            ArrayList<AprilTagDetection> tags = instancia.returnTagProcessor1().getDetections();
            for(AprilTagDetection tag : tags){

                com.arcrobotics.ftclib.geometry.Pose2d rpose = Positioner.getRobotPose(tag, new Transform2d(new Translation2d(-6.5, 0), new Rotation2d(Math.toRadians(180))));
                measurePoses.add(rpose);
            }
        }

        if (instancia.returnTagProcessor2().getDetections().size() > 0){

            ArrayList<AprilTagDetection> tags = instancia.returnTagProcessor2().getDetections();

            for (AprilTagDetection tag : tags){

                com.arcrobotics.ftclib.geometry.Pose2d rpose = Positioner.getRobotPose(tag, new Transform2d(new Translation2d(-6.5, 0), new Rotation2d(Math.toRadians(0))));
                measurePoses.add(rpose);
            }
        }

        //obtençao do ângulo
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.RADIANS);

        //Chama o fitro e passa os valores (vel, statePose, measurePoses)
        com.arcrobotics.ftclib.geometry.Pose2d delPose = mecdrive.mecanumDeltaPose(
                encoderTicksToInches(leftFront.getCurrentPosition()),
                encoderTicksToInches(rightFront.getCurrentPosition()),
                encoderTicksToInches(leftRear.getCurrentPosition()),
                encoderTicksToInches(rightRear.getCurrentPosition()),
                verticalWheelDistance,
                lateralWheelDistance
        );

        if(measurePoses.size() > 0){
            kalmanPose.updateFilter(delPose, measurePoses, orientation.getYaw(AngleUnit.RADIANS));
        } else {
            double orient = orientation.getYaw(AngleUnit.RADIANS);
            kalmanPose.addDelta(delPose.getTranslation().rotateBy(new Rotation2d(orient)), orient);
        }
        filteredPose = kalmanPose.getPose();
    }
}
