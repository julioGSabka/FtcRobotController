package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class OpmodeCV extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx motorFrontLeft = null;
    private DcMotorEx motorBackLeft = null;
    private DcMotorEx motorFrontRight = null;
    private DcMotorEx motorBackRight = null;
    private DcMotorEx Intake = null;
    private DcMotorEx Lift = null;
    private Servo garra = null;
    private ServoImplEx cotovelo = null;
    private ServoImplEx ombroR = null;
    private ServoImplEx ombroL = null;
    private DistanceSensor distanceSensor = null;
    private ColorSensor colorSensor = null;

    private int RB_presses = 0;
    private int PixelsnaGarra = 0;
    private double distanciaAnterior = 0;
    private double valorRed = 0;
    private double valorGreen = 0;
    private double valorBlue = 0;
    private String corPixel_1 = null;
    private String corPixel_2 = null;

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //HardwareMap Config
        //Motors
        motorFrontLeft = hardwareMap.get(DcMotorEx.class,"motorFrontLeft"); //2
        motorBackLeft = hardwareMap.get(DcMotorEx.class,"motorBackLeft"); //3
        motorFrontRight = hardwareMap.get(DcMotorEx.class,"motorFrontRight"); //0
        motorBackRight = hardwareMap.get(DcMotorEx.class,"motorBackRight"); //1
        Intake = hardwareMap.get(DcMotorEx.class,"Intake"); //Ex0
        Lift = hardwareMap.get(DcMotorEx.class, "Lift"); //Ex1

        colorSensor = hardwareMap.get(ColorSensor.class, "ColorSensor"); //2
        distanceSensor = hardwareMap.get(DistanceSensor.class, "DistanceSensor");

        //Servos
        garra = hardwareMap.get(Servo.class, "garra"); //Ex0
        cotovelo = hardwareMap.get(ServoImplEx.class, "cotovelo"); //4
        ombroR = hardwareMap.get(ServoImplEx.class, "ombroR"); //2
        ombroL = hardwareMap.get(ServoImplEx.class, "ombroL"); //0

        //Configure Motors
        //motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setDirection(DcMotorEx.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorEx.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotorEx.Direction.REVERSE);


        
        Intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Lift.setDirection(DcMotorSimple.Direction.REVERSE);

        distanciaAnterior = distanceSensor.getDistance(DistanceUnit.CM);

        garra.setPosition(0);
        cotovelo.setPosition(0.6);
        ombroL.setPosition(0.03);
        ombroR.setPosition(0.97);
        sleep(1000);
        cotovelo.setPosition(0.47);
        sleep(1500);
        DisableServos();

        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");

            valorRed = colorSensor.red();
            valorGreen = colorSensor.green();
            valorBlue = colorSensor.blue();

            double velocity = (gamepad1.right_trigger * 0.70) + 0.20;
            double y = gamepad1.left_stick_y * velocity;
            double x = gamepad1.left_stick_x * -1.1 * velocity;
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

            if (gamepad2.a == true) {
                EnableServos();
                cotovelo.setPosition(1);
                sleep(1000);
                ombroL.setPosition(1);
                ombroR.setPosition(0);
                sleep(500);
                cotovelo.setPosition(0);
            }
            if (gamepad2.b == true) {
                EnableServos();
                cotovelo.setPosition(0.6);
                sleep(500);
                ombroL.setPosition(0.03);
                ombroR.setPosition(0.97);
                sleep(1000);
                cotovelo.setPosition(0.47);
                DisableServos();
            }

            if (gamepad2.dpad_up){
                Lift.setVelocity(700);
                //Lift.setVelocityPIDFCoefficients(6.5,0.65,0,65);
                Lift.setVelocityPIDFCoefficients(6,0,0,45);
                Lift.setTargetPosition(2000);
                Lift.setTargetPositionTolerance(10);
                Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if (gamepad2.dpad_down){
                Lift.setVelocity(700);
                //Lift.setVelocityPIDFCoefficients(6.5,0.65,0,65);
                Lift.setVelocityPIDFCoefficients(6,0,0,45);
                Lift.setTargetPosition(0);
                Lift.setTargetPositionTolerance(10);
                Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            if (gamepad2.right_bumper) {
                RB_presses += 1;
                if (RB_presses == 3) { //Se apertar 3 vezes, faz a mesma coisa que 1 vez
                    RB_presses = 1;
                }

                //Fazer a garra soltar os pixels
                if (RB_presses == 1) {
                    garra.setPosition(0.6);
                    PixelsnaGarra = 1;
                } else if (RB_presses == 2) {
                    garra.setPosition(0);
                    PixelsnaGarra = 0;
                }
            }

            if (gamepad2.left_bumper) {
                garra.setPosition(1);
            }

            if (gamepad2.start) {
                Intake.setPower(-10);
                PixelsnaGarra = 0;
                corPixel_1 = null;
                corPixel_2 = null;
            }

            if (gamepad2.back) {
                Intake.setPower(0);
            }

            if (distanceSensor.getDistance(DistanceUnit.CM) < 5.5 && distanciaAnterior > 5.5) {
                PixelsnaGarra += 1;
                if (PixelsnaGarra == 2) {
                    sleep(700);
                    Intake.setPower(0);
                }
                valorRed = colorSensor.red();
                valorGreen = colorSensor.green();
                valorBlue = colorSensor.blue();

                if (valorBlue > 90) {
                    if (PixelsnaGarra == 1) {
                        corPixel_1 = "Roxo";
                    } else {
                        corPixel_2 = "Roxo";
                    }
                }
                if (valorGreen > 120) {
                    if (PixelsnaGarra == 1) {
                        corPixel_1 = "Verde";
                    } else {
                        corPixel_2 = "Verde";
                    }
                }
                if (valorRed < 70) {
                    if (PixelsnaGarra == 1) {
                        corPixel_1 = "Amarelo";
                    } else {
                        corPixel_2 = "Amarelo";
                    }
                }
            }

            distanciaAnterior = distanceSensor.getDistance(DistanceUnit.CM);

            telemetry.addLine("Opmode");
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addLine("============= Sistema SERVOS =============");
            telemetry.addData("OmbroR: ", ombroR.getPosition());
            telemetry.addData("OmbroL: ", ombroL.getPosition());
            telemetry.addData("Cotovelo: ", cotovelo.getPosition());
            telemetry.addData("Garra: ", garra.getPosition());
            telemetry.addLine("============= Sistema MOTORES ============");
            telemetry.addData("Intake: ", Intake.getPower());
            telemetry.addData("LiftCurrentPos:", Lift.getCurrentPosition());
            telemetry.addData("LiftTargetPos:", Lift.getTargetPosition());
            telemetry.addLine("============= Sistema SENSORES ===========");
            telemetry.addData("Distancia em cm: ", distanceSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("Pixels na garra: ", PixelsnaGarra);
            telemetry.addData("Valor Red: ", valorRed);
            telemetry.addData("Valor Green: ", valorGreen);
            telemetry.addData("Valor Blue: ", valorBlue);
            telemetry.addData("Cor do Pixel 1: ", corPixel_1);
            telemetry.addData("Cor do Pixel 2: ", corPixel_2);
            //telemetry.addData("Valor hexadecimal: ", colorSensor.argb());

            telemetry.update();
        }
    }

    public void DisableServos(){
        cotovelo.setPwmDisable();
        ombroR.setPwmDisable();
        ombroL.setPwmDisable();
    }

    public void EnableServos(){
        cotovelo.setPwmEnable();
        ombroR.setPwmEnable();
        ombroL.setPwmEnable();
    }
}
