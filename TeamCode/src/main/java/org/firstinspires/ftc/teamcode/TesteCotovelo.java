package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp
public class TesteCotovelo extends LinearOpMode {

    private ServoImplEx cotovelo = null;
    private ServoImplEx ombroR = null;
    private ServoImplEx ombroL = null;

    private double servoPos = 0;

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        cotovelo = hardwareMap.get(ServoImplEx.class, "cotovelo"); //4
        ombroR = hardwareMap.get(ServoImplEx.class, "ombroR"); //2
        ombroL = hardwareMap.get(ServoImplEx.class, "ombroL"); //0

        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");

            if (gamepad2.a) {
                servoPos = 0;
                cotovelo.setPosition(servoPos);
            }

            if (gamepad2.b) {
                servoPos = 1;
                cotovelo.setPosition(servoPos);
            }

            if (gamepad2.y) {
                servoPos = 0.5;
                cotovelo.setPosition(servoPos);
            }

            if (gamepad2.dpad_up) {
                servoPos = 0.72;
                cotovelo.setPosition(servoPos);
            }

            if (gamepad2.dpad_down) {
                servoPos = 0.4;
                cotovelo.setPosition(servoPos);
            }

            if (gamepad2.dpad_left) {
                ombroL.setPosition(0.05);
                ombroR.setPosition(0.95);
            }

            if (gamepad2.dpad_right) {
                ombroL.setPosition(0.9);
                ombroR.setPosition(0.1);
            }

            telemetry.addData("Pos", servoPos);
            telemetry.addData("ServoRealPos", cotovelo.getPosition());
            telemetry.addData("OmbroL Pos", ombroL.getPosition());
            telemetry.addData("OmbroR Pos", ombroR.getPosition());
            telemetry.update();

        }
    }
}
