package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.ElevationSystem;

@TeleOp
public class TesteElevacao extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private ElevationSystem elevation;

    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        elevation = new ElevationSystem(hardwareMap, true);

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
                elevation.UpRobot();
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

            elevation.joystickControl(gamepad1.left_stick_y, gamepad1.right_stick_y);

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
