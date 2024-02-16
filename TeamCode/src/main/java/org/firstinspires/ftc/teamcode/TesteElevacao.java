package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class TesteElevacao extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx motorFrontLeft = null;
    private DcMotorEx motorBackLeft = null;
    private DcMotorEx motorFrontRight = null;
    private DcMotorEx motorBackRight = null;
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
                servoElevacaoL.setPosition(1);
                servoElevacaoR.setPosition(0.36);
            }
            if (gamepad1.dpad_left) {
                motorElevacaoL.setPower(0.5);
                motorElevacaoR.setPower(0.5);
            }

            //Descer
            if (gamepad1.dpad_down) {
                servoElevacaoL.setPosition(0);
                servoElevacaoR.setPosition(0);
            }
            if (gamepad1.dpad_right) {
                motorElevacaoL.setPower(0);
                motorElevacaoR.setPower(0);
            }
            if (gamepad1.back) {
                motorElevacaoL.setPower(-0.5);
                motorElevacaoR.setPower(-0.5);
            }

            telemetry.addLine("Opmode");
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("MotorElevacaoL:", motorElevacaoL.getPower());
            telemetry.addData("MotorElevacaoR:", motorElevacaoR.getPower());
            telemetry.update();
        }
    }

}
