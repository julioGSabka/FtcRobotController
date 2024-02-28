package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
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

    private int posGarra = 0;
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

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        launcher = hardwareMap.get(Servo.class, "launcher");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "DistanceSensor");//2

        //Configure Motors
        motorBackRight.setDirection(DcMotorEx.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotorEx.Direction.REVERSE);

        distanciaAnterior = distanceSensor.getDistance(DistanceUnit.CM);

        boolean lastPress = false;
        boolean startElevation = false;

        arm.DownArm();
        arm.openGarra();

        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");

            if (gamepad2.a) {
                arm.UpArm();
            }
            if (gamepad2.b) {
                arm.DownArm();
            }

            //Operação da Garra
            if (gamepad2.left_bumper) { //abre a garra
                if(!lastPress) {
                    //se RISING EDGE (ou seja false -> true)

                    if (posGarra < 2) { //Se apertar 2 vezes, pare de mudar
                        posGarra ++;
                    }

                    //Fazer a garra soltar os pixels
                    if (posGarra == 1) {
                        arm.midlleGarra();
                        PixelsnaGarra = 1;
                    } else if (posGarra == 2) {
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
                posGarra = 0;
            }

            //Intake Commands
            if (gamepad2.start) {
                intake.startIntake();
                //PixelsnaGarra = 0;
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
                startElevation = true;
            }
            if (startElevation){
                if (gamepad2.x){
                    elevation.TensionCord();
                }
                if (gamepad2.y){
                    elevation.liftUpRobot();
                }
            }

            //Drone Launcher
            if (gamepad1.right_bumper){
                launcher.setPosition(1);
            }
            if (gamepad1.left_bumper){
                launcher.setPosition(0);
            }

            //Automatic Intake
            if (distanceSensor.getDistance(DistanceUnit.CM) < 10 && distanciaAnterior > 10) {
                PixelsnaGarra += 1;
                if (PixelsnaGarra == 2) {
                    sleep(700);
                    intake.stopIntake();
                }
            }
            distanciaAnterior = distanceSensor.getDistance(DistanceUnit.CM);

            //Alinhamento Automatico com as AprilTags
            if (gamepad1.dpad_left) {
                moveRobot(instancia.AlignToBackdropTag(1, 4));
            }else if (gamepad1.dpad_down) {
                moveRobot(instancia.AlignToBackdropTag(2, 5));
            } else if (gamepad1.dpad_right) {
                moveRobot(instancia.AlignToBackdropTag(3, 6));
            } else if (gamepad2.right_stick_button){
                intake.startIntake();
                while(intake.getIntakeCurrent() < 3){
                    motorFrontLeft.setPower(-0.2);
                    motorBackLeft.setPower(-0.2);
                    motorFrontRight.setPower(-0.2);
                    motorBackRight.setPower(-0.2);
                }
                sleep(0);
                //intake.stopIntake();
                motorFrontLeft.setPower(0.3);
                motorBackLeft.setPower(0.3);
                motorFrontRight.setPower(0.3);
                motorBackRight.setPower(0.3);
                sleep(1000);
                motorFrontLeft.setPower(0);
                motorBackLeft.setPower(0);
                motorFrontRight.setPower(0);
                motorBackRight.setPower(0);
            } else{
                double velocity = (gamepad1.right_trigger * 0.70) + 0.20;
                double y = -gamepad1.left_stick_y * velocity;
                double x = gamepad1.left_stick_x * 1.1 * velocity;
                double rx = -gamepad1.right_stick_x * velocity;

                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                double frontLeftPower = (y + x + rx) / denominator;
                double backLeftPower = (y - x + rx) / denominator;
                double frontRightPower = (y - x - rx) / denominator;
                double backRightPower = (y + x - rx) / denominator;

                motorFrontLeft.setPower(frontLeftPower);
                motorBackLeft.setPower(backLeftPower);
                motorFrontRight.setPower(frontRightPower);
                motorBackRight.setPower(backRightPower);
            }

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
