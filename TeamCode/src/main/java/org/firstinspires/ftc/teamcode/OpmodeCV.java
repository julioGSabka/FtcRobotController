package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystems.ArmSystem;
import org.firstinspires.ftc.teamcode.Subsystems.ElevationSystem;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSystem;
import org.firstinspires.ftc.teamcode.vision.InitPipes;

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

    private ArmSystem arm;
    private ElevationSystem elevation;
    private IntakeSystem intake;
    InitPipes instancia = InitPipes.getInstancia();

    private int LB_presses = 0;
    private int PixelsnaGarra = 0;

    //Distance-Color Sensor Variables
    private double distanciaAnterior = 0;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //HardwareMap Config
        arm = new ArmSystem(hardwareMap);
        elevation = new ElevationSystem(hardwareMap);
        intake = new IntakeSystem(hardwareMap);
        instancia.initVision(hardwareMap);
        instancia.activateTFODProcessor(false);

        motorFrontLeft = hardwareMap.get(DcMotorEx.class,"motorFrontLeft"); //0
        motorBackLeft = hardwareMap.get(DcMotorEx.class,"motorBackLeft"); //1
        motorFrontRight = hardwareMap.get(DcMotorEx.class,"motorFrontRight"); //2
        motorBackRight = hardwareMap.get(DcMotorEx.class,"motorBackRight"); //3
        launcher = hardwareMap.get(Servo.class, "launcher");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "DistanceSensor");//2

        //Configure Motors
        motorBackRight.setDirection(DcMotorEx.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotorEx.Direction.REVERSE);

        distanciaAnterior = distanceSensor.getDistance(DistanceUnit.CM);

        boolean lastPress = false;

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
                    //Arm Operation
                    if (gamepad2.a) {
                        arm.UpArm();
                    }
                    if (gamepad2.b) {
                        arm.DownArm();
                    }
                }
            }).start();

            //Operação da Garra
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

            //Intake Commands
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

            //Elevação do robô
            //Subir ganchos
            if (gamepad2.dpad_up) {
                elevation.UpGanchos();
            }
            //Descer ganchos
            if (gamepad2.dpad_down) {
                elevation.DownGanchos();
            }
            elevation.joystickControl(gamepad2.right_stick_y, gamepad2.left_stick_y);

            //Drone Launcher
            if (gamepad1.right_bumper){
                launcher.setPosition(1);
            }

            //Automatic Intake
            if (distanceSensor.getDistance(DistanceUnit.CM) < 5.5 && distanciaAnterior > 5.5) {
                PixelsnaGarra += 1;
                if (PixelsnaGarra == 2) {
                    sleep(700);
                    intake.stopIntake();
                }
            }
            distanciaAnterior = distanceSensor.getDistance(DistanceUnit.CM);

            new Thread(new Runnable() {
                @Override
                public void run() {
                    //Alinhamento Automatico com as AprilTags
                    if (gamepad1.dpad_left) {
                        moveRobot(instancia.AlignToBackdropTag(1, 4));
                    }
                    if (gamepad1.dpad_down) {
                        moveRobot(instancia.AlignToBackdropTag(2, 5));
                    }
                    if (gamepad1.dpad_right) {
                        moveRobot(instancia.AlignToBackdropTag(3, 6));
                    }
                }
            }).start();


            telemetry.addLine("Opmode");
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addLine("============= Sistema SERVOS =============");
            telemetry.addData("Ombro: ", arm.getOmbroPos());
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

        instancia.closeCams();
    }

    public void moveRobot(List<Double> vels) {
        // Send powers to the wheels.
        motorFrontLeft.setPower(vels.get(0));
        motorFrontRight.setPower(vels.get(1));
        motorBackLeft.setPower(vels.get(2));
        motorBackRight.setPower(vels.get(3));
    }

}
