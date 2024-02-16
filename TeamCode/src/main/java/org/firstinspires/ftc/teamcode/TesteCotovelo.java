package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp
public class TesteCotovelo extends LinearOpMode {

    private ServoImplEx cotovelo = null;
    private ServoImplEx ombroR = null;

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        cotovelo = hardwareMap.get(ServoImplEx.class, "cotovelo"); //4
        ombroR = hardwareMap.get(ServoImplEx.class, "ombroR"); //2

        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");

            if (gamepad2.a) {
                //cotovelo.setPosition(0);
            }

            if (gamepad2.b) {
                cotovelo.setPosition(1);
            }

            if (gamepad2.y) {
                cotovelo.setPosition(0.5);
            }

            if (gamepad2.dpad_up) {
                cotovelo.setPosition(0.69);
            }

            if (gamepad2.dpad_down) {
                //cotovelo.setPosition(0.25);
            }

            if (gamepad2.dpad_left) {
                ombroR.setPosition(0.87);
            }

            if (gamepad2.dpad_right) {
                ombroR.setPosition(0);
            }

            telemetry.addData("ServoRealPos", cotovelo.getPosition());
            telemetry.addData("OmbroR Pos", ombroR.getPosition());
            telemetry.update();

        }
    }
}
