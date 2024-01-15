package org.firstinspires.ftc.teamcode.RoadRunnerScripts.drive;

import android.util.Size;

import androidx.annotation.Nullable;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.AprilTagCustomDatabase;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.jetbrains.annotations.NotNull;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Config
public class AprilTagsLocalizer implements Localizer {

    private RealMatrix x;  // Estado do sistema
    private RealMatrix P;  // Matriz de covariância do estado
    private RealMatrix F;  // Matriz de transição de estado
    private RealMatrix H;  // Matriz de observação
    private RealMatrix R;  // Matriz de covariância da medida
    private RealMatrix I;

    private ArrayList<AprilTagDetection> up;

    public static double TICKS_PER_REV = 537.6;
    public static double WHEEL_RADIUS = 1.47; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    private DcMotorEx motorFrontLeft, motorBackLeft, motorFrontRight, motorBackRight = null;
    private List<DcMotorEx> motors;

    private List<Integer> lastEncPositions = new ArrayList<>();
    private List<Integer> lastEncVels = new ArrayList<>();
    private List<Double> ThetasValues = new ArrayList<>();

    private static int EncmotorFrontLeft, EncmotorBackLeft, EncmotorFrontRight, EncmotorBackRight = 0;

    private long lastTime;

    private static Pose2d mPoseEstimate = new Pose2d();
    private Pose2d rawPose = new Pose2d();
    private double PoseX = 0;
    private double PoseY = 0;
    private double Orientation = 0;
    private int BestTag = 0;

    private ArrayList<AprilTagDetection> detections = new ArrayList<AprilTagDetection>();

    AprilTagProcessor tagProcessor;
    VisionPortal visionPortal;
    ArrayList<AprilTagDetection> tag;

    public AprilTagsLocalizer(HardwareMap hardwareMap) {
        new AprilTagsLocalizer(hardwareMap, true);
    }

    public AprilTagsLocalizer(HardwareMap hardwareMap, boolean resetPos) {

        motorFrontLeft = hardwareMap.get(DcMotorEx.class,"motorFrontLeft"); //0
        motorBackLeft = hardwareMap.get(DcMotorEx.class,"motorBackLeft"); //1
        motorFrontRight = hardwareMap.get(DcMotorEx.class,"motorFrontRight"); //2
        motorBackRight = hardwareMap.get(DcMotorEx.class,"motorBackRight"); //3
        motors = Arrays.asList(motorFrontLeft, motorBackLeft, motorBackRight, motorFrontRight);

        mPoseEstimate = new Pose2d();
        rawPose = new Pose2d();

        //Declara e starta a câmera
        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setLensIntrinsics(822.317, 822.317, 319.495, 242.502)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagCustomDatabase.getCenterStageLibrary())
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(tagProcessor)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();

        visionPortal.setProcessorEnabled(tagProcessor, true);

    }

    @NotNull
    @Override
    public Pose2d getPoseEstimate() {

        double[][] EncoderPose = {{EncoderPoseEstimate().getX(), EncoderPoseEstimate().getY(), EncoderPoseEstimate().getHeading()}};
        RealMatrix measurementFromEncoder = MatrixUtils.createRealMatrix(EncoderPose);
        double[][] VisionPose = {{AprilTagPoseEstimate().getX(), AprilTagPoseEstimate().getY(), AprilTagPoseEstimate().getHeading()}};
        RealMatrix measurementFromAprilTag = MatrixUtils.createRealMatrix(VisionPose);

        KalmanFilter();
        predict();
        RealMatrix estimatedPose = correct(measurementFromEncoder, measurementFromAprilTag, tag.get(BestTag).decisionMargin);
        Pose2d finalPose = new Pose2d(estimatedPose.getEntry(0, 0), estimatedPose.getEntry(0, 1), estimatedPose.getEntry(0, 2));

        return finalPose;
    }

    @Override
    public void setPoseEstimate(@NotNull Pose2d pose2d) {
        mPoseEstimate = pose2d;
    }

    public static double getHeading() {
        return mPoseEstimate.getHeading();
    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {

        List<Integer> lastEncoder = lastEncVels;
        lastEncVels.clear();
        ThetasValues.clear();

        long currentTime = System.currentTimeMillis();
        long deltaTime = currentTime - lastTime;

        List<Double> wheelVelocities = new ArrayList<>();
        int i = 0;
        for (DcMotorEx motor : motors) {
            int vel = (int) motor.getVelocity();
            lastEncVels.add(vel);
            wheelVelocities.add(encoderTicksToInches(vel));
            double DeltasThetas = calcularDeltaTheta(encoderTicksToInches(vel), lastEncoder.get(i), deltaTime);
            ThetasValues.add(DeltasThetas);
            i += 1;
        }

        double vx = WHEEL_RADIUS * (ThetasValues.get(0) / 2 - ThetasValues.get(1) / 2 - ThetasValues.get(2) / 2 + ThetasValues.get(3) / 2);
        double vy = WHEEL_RADIUS * (ThetasValues.get(0) / 2 + ThetasValues.get(1) / 2 - ThetasValues.get(2) / 2 - ThetasValues.get(3) / 2);
        double omega = WHEEL_RADIUS * (ThetasValues.get(0) / (2 * WHEEL_RADIUS) + ThetasValues.get(1) / (2 * WHEEL_RADIUS) - ThetasValues.get(2) / (2 * WHEEL_RADIUS) - ThetasValues.get(3) / (2 * WHEEL_RADIUS));

        ChassisSpeeds velocity = new ChassisSpeeds(vx, vy, omega);

        return new Pose2d(velocity.vxMetersPerSecond /.0254,velocity.vyMetersPerSecond /.0254,velocity.omegaRadiansPerSecond);

    }

    @Override
    public void update() {
        up = tagProcessor.getDetections();
        float poseConfidence = tag.get(BestTag).decisionMargin;
    }


    private double calcularDeltaTheta(double currentEncoder, double lastEncoder, long deltaTime) {
        double ticksToRadians = 2 * Math.PI / 360.0;
        return (currentEncoder - lastEncoder) * ticksToRadians / deltaTime;
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    private double norm(double angle) {
        while (angle>Math.toRadians(360)) angle-=Math.toRadians(360);
        while (angle<=0) angle+=Math.toRadians(360);
        return angle;
    }

    public void KalmanFilter() {
        int stateSize = 3;  // X, Y, direção
        int measurementSize = 3;  // Medidas X, Y, direção

        x = MatrixUtils.createRealMatrix(stateSize, 1);
        P = MatrixUtils.createRealMatrix(stateSize, stateSize);
        F = MatrixUtils.createRealMatrix(stateSize, stateSize);
        H = MatrixUtils.createRealMatrix(measurementSize, stateSize);
        R = MatrixUtils.createRealMatrix(measurementSize, measurementSize);
        I = MatrixUtils.createRealIdentityMatrix(stateSize);
    }

    public void predict() {
        // x = F * x
        // P = F * P * F^T
        x = F.multiply(x);
        P = F.multiply(P).multiply(F.transpose());
    }

    public RealMatrix correct(RealMatrix measurementFromEncoder, RealMatrix measurementFromAprilTag, double confidence) {
        // Implemente a etapa de correção do filtro de Kalman usando a posição da tag
        // K = P * H^T * (H * P * H^T + R)^(-1)
        // x = x + K * (measurementFromAprilTag - H * x)
        // P = (I - K * H) * P

        // Ajuste a influência da confiança da fonte na matriz R
        R = R.scalarMultiply(confidence);

        RealMatrix K = P.multiply(H.transpose()).multiply(
                MatrixUtils.inverse(H.multiply(P).multiply(H.transpose()).add(R)));

        x = x.add(K.multiply(measurementFromAprilTag.subtract(H.multiply(x))));
        P = I.subtract(K.multiply(H)).multiply(P);

        // Aplique a correção da medida do encoder
        x = x.add(measurementFromEncoder);

        return x;
    }

    public Pose2d AprilTagPoseEstimate(){

        tagProcessor.setPoseSolver(AprilTagProcessor.PoseSolver.OPENCV_SOLVEPNP_EPNP);

        if (tagProcessor.getDetections().size() > 0){

            tag = tagProcessor.getDetections();
            int detectionNumber = tag.size();

            if (detectionNumber > 1) {
                BestTag = tag.get(0).id; // Inicializa BestTag com o primeiro elemento

                for (int i = 1; i < detectionNumber; i++) {
                    if (tag.get(i).decisionMargin > tag.get(i - 1).decisionMargin) {
                        BestTag = tag.get(i).id;
                    }
                }
            } else {
                BestTag = (detectionNumber == 1) ? tag.get(0).id : 0;
            }

            PoseX = tag.get(BestTag).metadata.fieldPosition.get(0) + tag.get(BestTag).ftcPose.y;
            PoseY = tag.get(BestTag).metadata.fieldPosition.get(1) + tag.get(BestTag).ftcPose.x;
            Orientation = -tag.get(BestTag).metadata.fieldPosition.get(3) + 180 + (tag.get(BestTag).ftcPose.bearing + (Math.atan(tag.get(BestTag).ftcPose.x / tag.get(BestTag).ftcPose.y)));

            //The T265's unit of measurement is meters. Dividing it by .0254 converts meters to inches.
            rawPose = new com.acmerobotics.roadrunner.geometry.Pose2d(PoseX /.0254, PoseY / .0254, norm(Math.toRadians(Orientation)));
            mPoseEstimate = rawPose; //offsets the pose to be what the pose estimate is;
        } else {
            RobotLog.v("NULL Tags Detections");
        }

        return mPoseEstimate;
    }

    public Pose2d EncoderPoseEstimate(){

        for (DcMotorEx motor : motors) {
            int CurrentPos = (int) motor.getCurrentPosition();
            lastEncPositions.add(CurrentPos);
        }

        // Atualizar os encoders
        EncmotorFrontLeft += lastEncPositions.get(0) / 384.5;
        EncmotorBackLeft += lastEncPositions.get(1) / 384.5;
        EncmotorBackRight += lastEncPositions.get(2) / 384.5;
        EncmotorFrontRight += lastEncPositions.get(3) / 384.5;

        // Calcular o deslocamento total
        double totalDisplacement = ((lastEncPositions.get(0) + lastEncPositions.get(1) + lastEncPositions.get(2) + lastEncPositions.get(3))/ 4.0) * (2 * Math.PI * WHEEL_RADIUS);


        // Calcular o ângulo atual do robô
        double theta = totalDisplacement / WHEEL_RADIUS;

        // Calcular as coordenadas X e Y
        double x = WHEEL_RADIUS * Math.cos(theta);
        double y = WHEEL_RADIUS * Math.sin(theta);

        Pose2d EncoderPos = new Pose2d(x, y, theta);

        return EncoderPos;
    }

}