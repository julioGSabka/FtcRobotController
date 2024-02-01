package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Airplane extends LinearOpMode {

    private Servo launcher = null;
    @Override
    public void runOpMode() {

        launcher = hardwareMap.get(Servo.class, "launcher");

        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {

            if(gamepad1.a){
                launcher.setPosition(1);
            }

            if(gamepad1.b){
                launcher.setPosition(0);
            }

        }
    }

}
