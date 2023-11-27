package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class OpmodeCV extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    //private DcMotorEx motorFrontLeft = null;
    //private DcMotorEx motorBackLeft = null;
    //private DcMotorEx motorFrontRight = null;
    //private DcMotorEx motorBackRight = null;
    private Servo garra = null;
    private Servo cotovelo = null;
    private Servo ombroR = null;
    private Servo ombroL = null;
    //private CRServo bracoR = null;
    //private CRServo bracoL = null;
    private DcMotorEx Intake = null;
    private DistanceSensor distanceSensor = null;
    private NormalizedColorSensor colorSensor = null;

    private int RB_presses = 0;
    private int PixelsnaGarra = 0;
    private double distanciaAnterior = 0;
    private double valorRed = 0;
    private double valorGreen = 0;
    private double valorBlue = 0;

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //HardwareMap Config
        //Motors
        /*
        motorFrontLeft = hardwareMap.get(DcMotorEx.class,"motorFrontLeft"); //
        motorBackLeft = hardwareMap.get(DcMotorEx.class,"motorBackLeft"); //
        motorFrontRight = hardwareMap.get(DcMotorEx.class,"motorFrontRight"); //
        motorBackRight = hardwareMap.get(DcMotorEx.class,"motorBackRight"); //
        */
        Intake = hardwareMap.get(DcMotorEx.class,"Intake"); //0
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "DistanceSensor"); //2
        distanceSensor = ((DistanceSensor) colorSensor);

        //Servos
        garra = hardwareMap.get(Servo.class, "garra"); //Ex0
        cotovelo = hardwareMap.get(Servo.class, "cotovelo"); //4
        ombroR = hardwareMap.get(Servo.class, "ombroR"); //2
        ombroL = hardwareMap.get(Servo.class, "ombroL"); //0
        /*
        bracoL = hardwareMap.get(CRServo.class, "bracoL");
        bracoR = hardwareMap.get(CRServo.class, "bracoR");
        */

        //Configure Motors
        /*
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setDirection(DcMotorEx.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotorEx.Direction.REVERSE);
        */
        Intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        distanciaAnterior = distanceSensor.getDistance(DistanceUnit.CM);


        garra.setPosition(0);
        cotovelo.setPosition(0.6);
        ombroL.setPosition(0);
        ombroR.setPosition(1);
        sleep(1000);
        cotovelo.setPosition(0.45);
        //bracoR.setPower(0);
        //bracoL.setPower(0);


        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            /*
            double velocity = (gamepad1.right_trigger * 0.80) + 0.20;
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
            */
            if (gamepad2.a == true) {
                cotovelo.setPosition(0.25);
                sleep(1000);
                ombroL.setPosition(0.9);
                ombroR.setPosition(0.1);
                sleep(500);
                cotovelo.setPosition(1);
            }
            if (gamepad2.b == true) {
                cotovelo.setPosition(0.2);
                sleep(500);
                ombroL.setPosition(0.03);
                ombroR.setPosition(0.97);
                sleep(1000);
                cotovelo.setPosition(0.47);
            }

            if (gamepad2.right_bumper) {
                RB_presses += 1;
                if (RB_presses == 3) { //Se apertar 3 vezes, faz a mesma coisa que 1 vez
                    RB_presses = 1;
                }

                //Fazer a garra soltar os pixels
                if (RB_presses == 1) {
                    garra.setPosition(0.5);
                    PixelsnaGarra = 1;
                } else if (RB_presses == 2) {
                    garra.setPosition(0);
                    PixelsnaGarra = 0;
                }
            }

            if (gamepad2.left_bumper) {
                garra.setPosition(1);
            }

            /*
            if (gamepad2.dpad_up) {
                bracoL.setPower(1);
                bracoR.setPower(1);
            }
            */
            if (gamepad2.start) {
                Intake.setPower(-10);
                PixelsnaGarra = 0;
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
            }

            distanciaAnterior = distanceSensor.getDistance(DistanceUnit.CM);
            valorRed = colorSensor.getNormalizedColors().red * (colorSensor.getNormalizedColors().red + colorSensor.getNormalizedColors().green + colorSensor.getNormalizedColors().blue);
            valorGreen = colorSensor.getNormalizedColors().green * (colorSensor.getNormalizedColors().red + colorSensor.getNormalizedColors().green + colorSensor.getNormalizedColors().blue);
            valorBlue = colorSensor.getNormalizedColors().blue * (colorSensor.getNormalizedColors().red + colorSensor.getNormalizedColors().green + colorSensor.getNormalizedColors().blue);


            telemetry.addLine("Opmode");
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addLine("============= Sistema SERVOS =============");
            telemetry.addData("OmbroR: ", ombroR.getPosition());
            telemetry.addData("OmbroL: ", ombroL.getPosition());
            telemetry.addData("Cotovelo: ", cotovelo.getPosition());
            telemetry.addData("Garra: ", garra.getPosition());
            telemetry.addLine("============= Sistema MOTORES ============");
            telemetry.addData("Intake: ", Intake.getPower());
            //telemetry.addData("BraçoL: ", bracoL.getPower());
            //telemetry.addData("BraçoR: ", bracoR.getPower());
            telemetry.addLine("============= Sistema SENSORES ===========");
            telemetry.addData("Distancia em cm: ", distanceSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("Pixels na garra: ", PixelsnaGarra);
            telemetry.addData("Valor Red: ", valorRed);
            telemetry.addData("Valor Green: ", valorGreen);
            telemetry.addData("Valor Blue: ", valorBlue);
            telemetry.addData("Valor hexadecimal: ", colorSensor.getNormalizedColors());
            telemetry.update();




        }
    }
}
