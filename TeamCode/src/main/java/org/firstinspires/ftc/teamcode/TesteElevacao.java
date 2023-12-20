package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp
public class TesteElevacao extends LinearOpMode {
    private ServoImplEx servoElevacaoL = null;
    private ServoImplEx servoElevacaoR = null;
    private DcMotorEx motorElevacaoL = null;
    private DcMotorEx motorElevacaoR = null;

    @Override
    public void runOpMode() {

        motorElevacaoL = hardwareMap.get(DcMotorEx.class, "motorElevacaoL"); //Ex2
        motorElevacaoR = hardwareMap.get(DcMotorEx.class, "motorElevacaoR"); //Ex3
        servoElevacaoL = hardwareMap.get(ServoImplEx.class, "servoElevacaoL"); //Ex2
        servoElevacaoR = hardwareMap.get(ServoImplEx.class, "servoElevacaoR"); //Ex4

        motorElevacaoL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorElevacaoR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {
            //Subir
            if (gamepad1.dpad_up) {
                servoElevacaoL.setPosition(0);
                servoElevacaoR.setPosition(0);
            }
            if (gamepad1.dpad_left) {
                motorElevacaoL.setPower(10);
                motorElevacaoR.setPower(10);
            }

            //Descer
            if (gamepad1.dpad_down) {
                servoElevacaoL.setPosition(1);
                servoElevacaoR.setPosition(1);
            }
            if (gamepad1.dpad_right) {
                motorElevacaoL.setPower(0);
                motorElevacaoR.setPower(0);
            }
        }
    }

}
