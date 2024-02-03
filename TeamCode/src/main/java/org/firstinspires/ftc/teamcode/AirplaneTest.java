package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class AirplaneTest extends LinearOpMode {

    Servo launcher = null;
    @Override
    public void runOpMode() {

        launcher = hardwareMap.get(Servo.class, "launcher");

        waitForStart();
        resetRuntime();

        while (opModeIsActive()){
            telemetry.addData("Status", "Running");

            if (gamepad1.right_bumper){
                launcher.setPosition(1);
            }
            if (gamepad1.left_bumper){
                launcher.setPosition(0);
            }

            telemetry.addData("LauncherPos", launcher);
            telemetry.update();
        }
    }
}
