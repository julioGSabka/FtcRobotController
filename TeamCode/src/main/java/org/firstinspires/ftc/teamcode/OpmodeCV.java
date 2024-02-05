package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystems.ArmSystem;
import org.firstinspires.ftc.teamcode.Subsystems.ElevationSystem;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSystem;
import org.firstinspires.ftc.teamcode.vision.AprilTagCustomDatabase;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp
public class OpmodeCV extends LinearOpMode {

    //Component Declaration
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx motorFrontLeft = null;
    private DcMotorEx motorBackLeft = null;
    private DcMotorEx motorFrontRight = null;
    private DcMotorEx motorBackRight = null;
    private Servo launcher = null;
    private DistanceSensor distanceSensor = null;
    private ColorSensor colorSensor = null;

    private ArmSystem arm;
    private ElevationSystem elevation;
    private IntakeSystem intake;

    private int LB_presses = 0;
    private int PixelsnaGarra = 0;

    //Distance-Color Sensor Variables
    private double distanciaAnterior = 0;

    //Vision Variables
    AprilTagProcessor tagProcessor1;
    AprilTagProcessor tagProcessor2;
    VisionPortal visionPortal1;
    VisionPortal visionPortal2;

    boolean targetFound = false;
    AprilTagDetection desiredTag = null;
    int desiredBlueTagID = -1;
    int desiredRedTagID = -1;

    final double DESIRED_DISTANCE = 12.0;
    final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED  = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN   = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    double drive = 0;    // Desired forward power/speed (-1 to +1)
    double strafe = 0;   // Desired strafe power/speed (-1 to +1)
    double turn = 0;     // Desired turning power/speed (-1 to +1)

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //HardwareMap Config
        arm = new ArmSystem(hardwareMap);
        elevation = new ElevationSystem(hardwareMap);
        intake = new IntakeSystem(hardwareMap);

        motorFrontLeft = hardwareMap.get(DcMotorEx.class,"motorFrontLeft"); //0
        motorBackLeft = hardwareMap.get(DcMotorEx.class,"motorBackLeft"); //1
        motorFrontRight = hardwareMap.get(DcMotorEx.class,"motorFrontRight"); //2
        motorBackRight = hardwareMap.get(DcMotorEx.class,"motorBackRight"); //3
        launcher = hardwareMap.get(Servo.class, "launcher");
        colorSensor = hardwareMap.get(ColorSensor.class, "ColorSensor"); //2
        distanceSensor = hardwareMap.get(DistanceSensor.class, "DistanceSensor");


        //Configure Motors
        motorBackRight.setDirection(DcMotorEx.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotorEx.Direction.REVERSE);

        distanciaAnterior = distanceSensor.getDistance(DistanceUnit.CM);

        boolean lastPress = false;

        initTags();

        arm.DownArm();
        arm.openGarra();

        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");

            double velocity = (gamepad1.right_trigger * 0.70) + 0.20;
            double y = gamepad1.left_stick_y * velocity;
            double x = gamepad1.left_stick_x * -1.1 * velocity;
            double rx = gamepad1.right_stick_x * velocity;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);

            new Thread(new Runnable() {
                @Override
                public void run() {
                    if (gamepad2.a == true) {
                        arm.UpArm();
                    }
                    if (gamepad2.b == true) {
                        arm.DownArm();
                    }
                }
            }).start();


            if (gamepad2.left_bumper) {
                if(!lastPress) {
                    LB_presses += 1;
                    if (LB_presses == 3) { //Se apertar 3 vezes, faz a mesma coisa que 1 vez
                        LB_presses = 1;
                    }

                    //Fazer a garra soltar os pixels
                    if (LB_presses == 1) {
                        arm.midlleGarra();
                        PixelsnaGarra = 1;
                    } else if (LB_presses == 2) {
                        arm.openGarra();
                        PixelsnaGarra = 0;
                    }
                }
                lastPress = true;
            }else{
                lastPress = false;
            }

            //Fecha
            if (gamepad2.right_bumper) {
                arm.closeGarra();
            }

            if (gamepad2.start) {
                intake.startIntake();
                PixelsnaGarra = 0;
            }

            if (gamepad2.back) {
                intake.stopIntake();
            }
            if (gamepad2.left_stick_button) {
                intake.reverseIntake();
            }

            if (distanceSensor.getDistance(DistanceUnit.CM) < 5.5 && distanciaAnterior > 5.5) {
                PixelsnaGarra += 1;
                if (PixelsnaGarra == 2) {
                    sleep(700);
                    intake.stopIntake();
                }
            }

            if (gamepad1.dpad_left) {
                desiredBlueTagID = 1;
                desiredRedTagID = 4;
                AlignToBackdropTag();
            }
            if (gamepad1.dpad_down) {
                desiredBlueTagID = 2;
                desiredRedTagID = 5;
                AlignToBackdropTag();
            }
            if (gamepad1.dpad_right) {
                desiredBlueTagID = 3;
                desiredRedTagID = 6;
                AlignToBackdropTag();
            }

            //Subir
            if (gamepad2.dpad_up) {
                elevation.UpGanchos();
            }
            //Descer
            if (gamepad2.dpad_down) {
                elevation.DownGanchos();
            }
            elevation.setMotorsPower(gamepad2.right_stick_y, gamepad2.left_stick_y);

            if (gamepad1.right_bumper){
                launcher.setPosition(1);
            }

            distanciaAnterior = distanceSensor.getDistance(DistanceUnit.CM);

            telemetry.addLine("Opmode");
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addLine("============= Sistema SERVOS =============");
            telemetry.addData("OmbroR: ", arm.getOmbroRPos());
            telemetry.addData("OmbroL: ", arm.getOmbroLPos());
            telemetry.addData("Cotovelo: ", arm.getCotoveloPos());
            telemetry.addData("Garra: ", arm.getGarraPos());
            telemetry.addLine("============= Sistema MOTORES ============");
            telemetry.addData("Intake: ", intake.getIntakePower());
            telemetry.addData("IntakeCurrent", intake.getIntakeCurrent());
            telemetry.addData("MotorElevacaoR:",elevation.getMotorsCurrent().get(0));
            telemetry.addData("MotorElevacaoL:",elevation.getMotorsCurrent().get(1));
            telemetry.addLine("============= Sistema MOTORES movimentação ============");
            telemetry.addData("MotorLeftBack:",motorBackLeft.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("MotorLeftFront:",motorFrontLeft.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("MotorRightBack:",motorBackRight.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("MotorRightFront:",motorFrontRight.getCurrent(CurrentUnit.AMPS));
            telemetry.addLine("============= Sistema SENSORES ===========");
            telemetry.addData("Distancia em CM: ", distanceSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("Pixels na garra: ", PixelsnaGarra);
            telemetry.update();
        }

        visionPortal1.stopStreaming();
        visionPortal1.stopLiveView();
        visionPortal1.close();
        visionPortal2.stopStreaming();
        visionPortal2.stopLiveView();
        visionPortal2.close();
    }

    public void initTags(){
        int[] portalsList = VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL);

        tagProcessor1 = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setLensIntrinsics(822.317, 822.317, 319.495, 242.502)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagCustomDatabase.getCenterStageLibrary())
                .build();

        tagProcessor2 = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setLensIntrinsics(1401.86, 1401.86, 659.724, 393.405)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagCustomDatabase.getCenterStageLibrary())
                .build();

        visionPortal1 = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(tagProcessor1)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .setLiveViewContainerId(portalsList[0])
                .setAutoStopLiveView(true)
                .build();

        visionPortal2 = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"))
                .addProcessor(tagProcessor2)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .setLiveViewContainerId(portalsList[1])
                .setAutoStopLiveView(true)
                .build();

        visionPortal1.setProcessorEnabled(tagProcessor1, true);
        visionPortal2.setProcessorEnabled(tagProcessor2, true);
    }

    public void AlignToBackdropTag() {


        List<AprilTagDetection> currentDetections = tagProcessor1.getDetections();
        for (AprilTagDetection detection : currentDetections) {

            if (detection.metadata != null) {

                if (detection.id == desiredBlueTagID || detection.id == desiredRedTagID) {
                    targetFound = true;
                    desiredTag = detection;
                    break;
                } else {
                    telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                }

            } else {
                telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
            }
        }

        // Tell the driver what we see, and what to do.
        if (targetFound) {
            telemetry.addData("\n>","HOLD Left-Bumper to Drive to Target\n");
            telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
            telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
            telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
            telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);
        } else {
            telemetry.addData("\n>","Drive using joysticks to find valid target\n");
        }

        // If Left Bumper is being pressed, AND we have found the desired target, Drive to target Automatically .
        if (targetFound) {

            // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
            double  rangeError      = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
            double  headingError    = desiredTag.ftcPose.bearing;
            double  yawError        = desiredTag.ftcPose.yaw;

            // Use the speed and turn "gains" to calculate how we want the robot to move.
            drive  = -Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
            strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

            telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
        }
        telemetry.update();

        // Apply desired axes motions to the drivetrain.
        moveRobot(drive, strafe, turn);
        sleep(10);
    }

    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        motorFrontLeft.setPower(leftFrontPower);
        motorFrontRight.setPower(rightFrontPower);
        motorBackLeft.setPower(leftBackPower);
        motorBackRight.setPower(rightBackPower);
    }


}
