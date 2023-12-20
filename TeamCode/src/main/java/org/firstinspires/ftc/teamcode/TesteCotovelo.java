package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp
public class TesteCotovelo extends LinearOpMode {

    private ServoImplEx cotovelo = null;

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        cotovelo = hardwareMap.get(ServoImplEx.class, "cotovelo"); //4

        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");

            if (gamepad2.a) {
                cotovelo.setPosition(0);
            }
            if (gamepad2.b) {
                cotovelo.setPosition(1);
            }
        }
    }
}
