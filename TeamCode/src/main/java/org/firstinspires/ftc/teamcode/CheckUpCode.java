package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.Arrays;
import java.util.List;

@TeleOp
public class CheckUpCode extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx motorFrontLeft = null;
    private DcMotorEx motorBackLeft = null;
    private DcMotorEx motorFrontRight = null;
    private DcMotorEx motorBackRight = null;
    private DcMotorEx Intake = null;
    private DcMotorEx motorElevacaoL = null;
    private DcMotorEx motorElevacaoR = null;
    private Servo garra = null;
    private Servo launcher = null;
    private ServoImplEx cotovelo = null;
    private ServoImplEx ombroR = null;
    private ServoImplEx ombroL = null;
    private ServoImplEx servoElevacaoL = null;
    private ServoImplEx servoElevacaoR = null;

    public static double RUNTIME = 2.0;

    private ElapsedTime timer;
    private double current = 0.0;

    private VoltageSensor batteryVoltageSensor;

    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //HardwareMap Config
        //Motors
        motorFrontLeft = hardwareMap.get(DcMotorEx.class,"motorFrontLeft"); //0
        motorBackLeft = hardwareMap.get(DcMotorEx.class,"motorBackLeft"); //1
        motorFrontRight = hardwareMap.get(DcMotorEx.class,"motorFrontRight"); //2
        motorBackRight = hardwareMap.get(DcMotorEx.class,"motorBackRight"); //3
        Intake = hardwareMap.get(DcMotorEx.class,"Intake"); //Ex0
        motorElevacaoL = hardwareMap.get(DcMotorEx.class, "motorElevacaoL"); //Ex2
        motorElevacaoR = hardwareMap.get(DcMotorEx.class, "motorElevacaoR"); //Ex3

        //Servos
        garra = hardwareMap.get(Servo.class, "garra"); //Ex0
        cotovelo = hardwareMap.get(ServoImplEx.class, "cotovelo"); //4
        ombroR = hardwareMap.get(ServoImplEx.class, "ombroR"); //2
        ombroL = hardwareMap.get(ServoImplEx.class, "ombroL"); //0
        servoElevacaoL = hardwareMap.get(ServoImplEx.class, "servoElevacaoL"); //Ex2
        servoElevacaoR = hardwareMap.get(ServoImplEx.class, "servoElevacaoR"); //Ex4
        launcher = hardwareMap.get(Servo.class, "launcher");

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        //Configure Motors
        motorBackRight.setDirection(DcMotorEx.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotorEx.Direction.REVERSE);
        motorElevacaoL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorElevacaoR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        List<DcMotorEx> motors = Arrays.asList(motorFrontLeft, motorBackLeft, motorFrontRight, motorBackRight,
                Intake);
        List<String> motoresString = Arrays.asList("motorFrontLeft", "motorBackLeft", "motorFrontRight", "motorBackRight", "Intake");

        waitForStart();
        resetRuntime();

        telemetry.addLine("Você deseja iniciar o código de checkup do robo?");
        telemetry.addLine(" - Pressione A para seguir em frente - ");
        telemetry.addLine("Deixe o robo suspenso do chão");
        telemetry.update();

        while (gamepad1.a != true){

        }

        telemetry.clear();
        telemetry.addLine("Iniciando o checkup");

        telemetry.addLine("VERIFICAÇÃO DA BATERIA");
        if (batteryVoltageSensor.getVoltage() < 11.7){
            telemetry.addData("Bateria Baixa", batteryVoltageSensor.getVoltage());
        } else {
            telemetry.addData("Bateria Utilizavel", batteryVoltageSensor.getVoltage());
        }
        telemetry.update();
        sleep(2000);

        telemetry.addLine("VERIFICAÇÃO DOS CABOS");
        int i = 0;
        for (DcMotorEx motor : motors){

            motor.setPower(0.2);
            sleep(200);
            if (motor.getVelocity() < 0.01){
                telemetry.addData("Motor", motoresString.get(i));
                telemetry.addData("** está desconectado", motor.getCurrent(CurrentUnit.AMPS));
            } else if (motor.getCurrent(CurrentUnit.AMPS) > 2){
                telemetry.addData("Motor", motoresString.get(i));
                telemetry.addData("** está consumindo além do esperado", motor.getCurrent(CurrentUnit.AMPS));
            } else {
                telemetry.addData("Motor", motoresString.get(i));
                telemetry.addData("está funcionando", motor.getCurrent(CurrentUnit.AMPS));
            }
            sleep(200);
            motor.setPower(0);

            i += 1;
        }
        telemetry.update();

        i = 0;
        telemetry.addLine("VERIFICAÇÃO DO CONSUMO");
        for (DcMotorEx motor : motors){
            motor.setPower(1);

            timer = new ElapsedTime();
            while (!isStopRequested() && timer.seconds() < RUNTIME) {
                current = Math.max(motor.getCurrent(CurrentUnit.AMPS), current);
            }
            if (current < 0.01){
                telemetry.addData("Motor", motoresString.get(i));
                telemetry.addData("** está desconectado", current);
            } else if (current > 4){
                telemetry.addData("Motor", motoresString.get(i));
                telemetry.addData("** está consumindo além do esperado", current);
            } else {
                telemetry.addData("Motor", motoresString.get(i));
                telemetry.addData("está funcionando", current);
            }
            sleep(2000);
            motor.setPower(0);

            i += 1;
        }
        telemetry.update();
        while (!isStopRequested()){

        }

    }

}
