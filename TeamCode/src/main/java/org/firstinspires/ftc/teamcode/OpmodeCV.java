package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class OpmodeCV extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx motorFrontLeft = null;
    private DcMotorEx motorBackLeft = null;
    private DcMotorEx motorFrontRight = null;
    private DcMotorEx motorBackRight = null;
    private Servo garra = null;
    private Servo cotovelo = null;
    private Servo ombroR = null;
    private Servo ombroL = null;
    private CRServo bracoR = null;
    private CRServo bracoL = null;
    private DcMotorEx Intake = null;

    private int RB_presses = 0;

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
        Intake = hardwareMap.get(DcMotorEx.class,"Intake");
        */

        //Servos
        garra = hardwareMap.get(Servo.class, "garra"); //1
        cotovelo = hardwareMap.get(Servo.class, "cotovelo"); //3
        ombroR = hardwareMap.get(Servo.class, "ombroR"); //2
        ombroL = hardwareMap.get(Servo.class, "ombroL"); //0
        bracoL = hardwareMap.get(CRServo.class, "bracoL");
        bracoR = hardwareMap.get(CRServo.class, "bracoR");

        //Configure Motors
        /*
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setDirection(DcMotorEx.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotorEx.Direction.REVERSE);
        Intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         */

        garra.setPosition(0);
        cotovelo.setPosition(0.5);
        sleep(2000);
        ombroL.setPosition(0);
        ombroR.setPosition(1);
        bracoR.setPower(0);
        bracoL.setPower(0);


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
            if (gamepad1.a == true) {
                cotovelo.setPosition(0.3);
                sleep(2000);
                ombroL.setPosition(1);
                ombroR.setPosition(0);
                cotovelo.setPosition(1);
            }
            if (gamepad1.b == true) {
                cotovelo.setPosition(0.3);
                ombroL.setPosition(0);
                ombroR.setPosition(1);
                sleep(2000);
                cotovelo.setPosition(0.5);
            }
            if (gamepad1.right_bumper) {
                RB_presses += 1;
                if (RB_presses == 3) { //Se apertar 3 vezes, faz a mesma coisa que 1 vez
                    RB_presses = 1;
                }

                //Fazer a garra soltar os pixels
                if (RB_presses == 1) {
                    garra.setPosition(0.5);
                } else if (RB_presses == 2) {
                    garra.setPosition(0);
                }
            }

            if (gamepad1.left_bumper) {
                garra.setPosition(1);
            }

            if (gamepad1.dpad_up) {
                bracoL.setPower(1);
                bracoR.setPower(1);
            }

            if (gamepad2.start) {
                Intake.setPower(500);
            }

            if (gamepad2.back) {
                Intake.setPower(0);
            }


            telemetry.addLine("Opmode");
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addLine("============= Sistema SERVOS =============");
            telemetry.addData("OmbroR: ", ombroR.getPosition());
            telemetry.addData("OmbroL: ", ombroL.getPosition());
            telemetry.addData("Cotovelo: ", cotovelo.getPosition());
            telemetry.addData("Garra: ", garra.getPosition());
            telemetry.addLine("============= Sistema MOTORES ============");
            telemetry.addData("Intake: ", Intake.getPower());
            telemetry.addData("BraçoL: ", bracoL.getPower());
            telemetry.addData("BraçoR: ", bracoR.getPower());
            telemetry.update();



        }
    }
}
