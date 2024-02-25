package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.vision.InitPipes;

import java.util.List;

@TeleOp
public class MoveTest extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx motorFrontLeft = null;
    private DcMotorEx motorBackLeft = null;
    private DcMotorEx motorFrontRight = null;
    private DcMotorEx motorBackRight = null;

    InitPipes instancia = InitPipes.getInstancia();

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        instancia.initVision(hardwareMap);
        instancia.activateTFODProcessor(false);

        motorFrontLeft = hardwareMap.get(DcMotorEx.class,"motorFrontLeft"); //0
        motorBackLeft = hardwareMap.get(DcMotorEx.class,"motorBackLeft"); //1
        motorFrontRight = hardwareMap.get(DcMotorEx.class,"motorFrontRight"); //2
        motorBackRight = hardwareMap.get(DcMotorEx.class,"motorBackRight"); //3

        motorFrontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorEx.Direction.REVERSE);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");

            if (gamepad1.a) {
                motorFrontLeft.setPower(0.5);
                motorBackLeft.setPower(0.5);
                motorFrontRight.setPower(0.5);
                motorBackRight.setPower(0.5);
            }

            if (gamepad1.b) {
                motorFrontLeft.setPower(0);
                motorBackLeft.setPower(0);
                motorFrontRight.setPower(0);
                motorBackRight.setPower(0);
            }

            if (gamepad1.dpad_left) {
                moveRobot(instancia.AlignToBackdropTag(1, 4));
            } else if (gamepad1.dpad_down) {
                moveRobot(instancia.AlignToBackdropTag(2, 5));
            } else if (gamepad1.dpad_right) {
                moveRobot(instancia.AlignToBackdropTag(3, 6));
            } else {
                double velocity = (gamepad1.right_trigger * 0.70) + 0.20;
                double y = -gamepad1.left_stick_y * velocity;
                double x = gamepad1.left_stick_x * 1.1 * velocity;
                double rx = -gamepad1.right_stick_x * velocity;

                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                double frontLeftPower = (y + x + rx) / denominator;
                double backLeftPower = (y - x + rx) / denominator;
                double frontRightPower = (y - x - rx) / denominator;
                double backRightPower = (y + x - rx) / denominator;

                motorFrontLeft.setPower(frontLeftPower);
                motorBackLeft.setPower(backLeftPower);
                motorFrontRight.setPower(frontRightPower);
                motorBackRight.setPower(backRightPower);
            }
        }
    }

    public void moveRobot(List<Double> vels) {
        // Send powers to the wheels.
        motorFrontLeft.setPower(vels.get(0));
        motorFrontRight.setPower(vels.get(1));
        motorBackLeft.setPower(vels.get(2));
        motorBackRight.setPower(vels.get(3));
    }
}
