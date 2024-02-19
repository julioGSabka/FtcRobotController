package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.vision.AprilTagCustomDatabase;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp
public class testeintakenovo extends LinearOpMode {

    private DcMotorEx Intake = null;
    private DcMotorEx Esteira = null;


    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //HardwareMap Config
        //Motors
        Intake = hardwareMap.get(DcMotorEx.class,"Intake"); //Ex0
        Esteira = hardwareMap.get(DcMotorEx.class, "Esteira"); //Ex1

        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");

            if (gamepad2.start) {
                Intake.setPower(-1);
                Esteira.setPower(0.9);
            }

            if (gamepad2.back) {
                Intake.setPower(0);
                Esteira.setPower(0);
            }
            if (gamepad2.left_stick_button) {
                Intake.setPower(-1);
            }

            telemetry.addLine("Opmode");
            telemetry.addLine("============= Sistema MOTORES ============");
            telemetry.addData("Intake: ", Intake.getPower());
            telemetry.addData("IntakeCurrent", Intake.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("LiftCurrentPos:", Esteira.getCurrent(CurrentUnit.AMPS));
        }
    }
}
