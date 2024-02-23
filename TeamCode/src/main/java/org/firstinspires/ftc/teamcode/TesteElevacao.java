package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.ElevationSystem;

@TeleOp
public class TesteElevacao extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotorEx motorFrontLeft = null;
    private DcMotorEx motorBackLeft = null;
    private DcMotorEx motorFrontRight = null;
    private DcMotorEx motorBackRight = null;

    private ElevationSystem elevation;

    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        elevation = new ElevationSystem(hardwareMap);

        motorFrontLeft = hardwareMap.get(DcMotorEx.class,"motorFrontLeft"); //0
        motorBackLeft = hardwareMap.get(DcMotorEx.class,"motorBackLeft"); //1
        motorFrontRight = hardwareMap.get(DcMotorEx.class,"motorFrontRight"); //2
        motorBackRight = hardwareMap.get(DcMotorEx.class,"motorBackRight"); //3

        motorBackRight.setDirection(DcMotorEx.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotorEx.Direction.REVERSE);

        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {

            //Subir
            if (gamepad1.dpad_up) {
                elevation.UpGanchos();
            }
            if (gamepad1.dpad_down) {
                elevation.DownGanchos();
            }

            if (gamepad1.y) {
                elevation.TensionCord();
            }

            if(gamepad1.x){
                elevation.liftUpRobot();
            }

            if (gamepad1.a) {
                elevation.StopMotors();
            }

            if (gamepad1.back) {
                elevation.ReverseMotors();
            }

            double velocity = (gamepad1.right_trigger * 0.70) + 0.20;
            double y = gamepad1.left_stick_y * velocity;
            double x = gamepad1.left_stick_x * -1.1 * velocity;
            double rx = gamepad1.right_stick_x * velocity;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);

            telemetry.addLine("Opmode");
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("MotorR CurrentPos:", elevation.getMotorsCurrentPosition().get(0));
            telemetry.addData("MotorL CurrentPos:", elevation.getMotorsCurrentPosition().get(1));
            telemetry.addData("MotorR TargetPos:", elevation.getMotorsTargetPosition().get(0));
            telemetry.addData("MotorL TargetPos:", elevation.getMotorsTargetPosition().get(1));
            telemetry.addData("MotorR Mode:", elevation.getMotorsMode().get(0));
            telemetry.addData("MotorL Mode:", elevation.getMotorsMode().get(1));
            telemetry.addData("MotorR Current", elevation.getMotorsCurrent().get(0));
            telemetry.addData("MotorL Current", elevation.getMotorsCurrent().get(1));
            telemetry.update();
        }
    }

}
