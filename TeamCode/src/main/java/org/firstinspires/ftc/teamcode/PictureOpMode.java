package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.vision.InitPipes;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.Locale;
@TeleOp
public class PictureOpMode extends LinearOpMode {
    InitPipes instancia = InitPipes.getInstancia();
    VisionPortal portal1;
    VisionPortal portal2;

    int frameCount1 = 0;
    int frameCount2 = 0;
    boolean lastA = false;
    boolean lastB = false;

    private DcMotorEx motorFrontLeft = null;
    private DcMotorEx motorBackLeft = null;
    private DcMotorEx motorFrontRight = null;
    private DcMotorEx motorBackRight = null;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        instancia.initVision(hardwareMap);
        portal1 = instancia.returnVisionPortal1();
        portal2 = instancia.returnVisionPortal2();

        motorFrontLeft = hardwareMap.get(DcMotorEx.class,"motorFrontLeft"); //0
        motorBackLeft = hardwareMap.get(DcMotorEx.class,"motorBackLeft"); //1
        motorFrontRight = hardwareMap.get(DcMotorEx.class,"motorFrontRight"); //2
        motorBackRight = hardwareMap.get(DcMotorEx.class,"motorBackRight"); //3

        motorFrontLeft .setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft  .setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight .setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorBackLeft.setDirection(DcMotorEx.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotorEx.Direction.REVERSE);

        waitForStart();

        telemetry.addLine("waiting for image");
        telemetry.update();

        while(opModeIsActive()){
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

            boolean a = gamepad1.a;
            boolean b = gamepad1.b;

            if(a && !lastA){
                portal1.saveNextFrameRaw(String.format(Locale.US, "RedragonCapture-%06d", frameCount1++));
            }
            lastA = a;

            if(b){
                portal2.saveNextFrameRaw(String.format(Locale.US, "LogiCapture-%06d", frameCount2++));
                sleep(300);
            }
            lastB = b;

            telemetry.addData("contagem Redragon", frameCount1);
            telemetry.addData("contagem Logi", frameCount2);
            telemetry.update();
        }
        instancia.closeCams();
    }
}
