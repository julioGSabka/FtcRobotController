package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.ArmSystem;

@TeleOp
public class TesteBraco extends LinearOpMode {

    ArmSystem arm = null;

    private int LB_presses = 0;
    boolean lastPress = false;

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        arm = new ArmSystem(hardwareMap);

        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");

            if (gamepad1.a){
                arm.setOmbro(0.87);
            }

            if (gamepad1.b){
                arm.setCotovelo(0.69);
            }

            if (gamepad1.y){
                arm.setOmbro(1);
            }

            if (gamepad1.x){
                arm.setCotovelo(1);
            }

            if (gamepad2.dpad_up) {
                arm.UpArm();
            }

            if (gamepad2.dpad_down) {
                arm.DownArm();
            }

            if (gamepad2.right_bumper){
                arm.closeGarra();
            }

            if (gamepad2.left_bumper) {
                if(!lastPress) {
                    LB_presses += 1;
                    if (LB_presses == 3) { //Se apertar 3 vezes, faz a mesma coisa que 1 vez
                        LB_presses = 1;
                    }

                    //Fazer a garra soltar os pixels
                    if (LB_presses == 1) {
                        arm.midlleGarra();
                    } else if (LB_presses == 2) {
                        arm.openGarra();
                    }
                }
                lastPress = true;
            }else{
                lastPress = false;
            }

            telemetry.addData("Cotovelo Pos", arm.getCotoveloPos());
            telemetry.addData("Ombro Pos", arm.getOmbroPos());
            telemetry.addData("Garra Pos", arm.getGarraPos());
            telemetry.update();

        }
    }
}
