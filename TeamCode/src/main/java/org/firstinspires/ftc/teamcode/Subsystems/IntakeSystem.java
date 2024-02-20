package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class IntakeSystem {

    private DcMotorEx intake = null;
    private DcMotorEx esteira = null;

    public IntakeSystem(HardwareMap hardwareMap) {
        new IntakeSystem(hardwareMap, true);
    }

    public IntakeSystem(HardwareMap hardwareMap, boolean using){
        intake = hardwareMap.get(DcMotorEx.class,"Intake"); //Ex0
        intake.setDirection(DcMotorEx.Direction.REVERSE);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        esteira = hardwareMap.get(DcMotorEx.class, "Esteira"); //
    }

    public void cuspirPixel(){
        intake.setPower(-0.7);
        esteira.setPower(-0.7);
        sleep(500);
        intake.setPower(0);
        esteira.setPower(0);
        sleep(200);
    }

    public void stopIntake(){
        intake.setPower(0);
        esteira.setPower(0);
    }

    public void startIntake(){
        intakeSetPower();
        esteiraSetPower();
    }

    public void intakeSetPower(){
       intake.setPower(0.7);
    }

    public void esteiraSetPower(){
        esteira.setPower(1);
    }

    public void reverseIntake(){
        intake.setPower(-1);
        esteira.setPower(-1);
    }

    public double getIntakePower(){
        return intake.getPower();
    }
    public double getEsteiraPower(){
        return esteira.getPower();
    }

    public double getIntakeCurrent(){
        return intake.getCurrent(CurrentUnit.AMPS);
    }

    public double getEsteiraCurrent(){
        return esteira.getCurrent(CurrentUnit.AMPS);
    }

    private void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
