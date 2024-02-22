package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        instancia.initVision(hardwareMap);
        portal1 = instancia.returnVisionPortal1();
        portal2 = instancia.returnVisionPortal2();

        waitForStart();

        telemetry.addLine("waiting for image");
        telemetry.update();

        while(opModeIsActive()){
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
