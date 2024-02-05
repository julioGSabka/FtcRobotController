package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class IntakeSystem {

    private DcMotorEx intake = null;

    public IntakeSystem(HardwareMap hardwareMap) {
        new IntakeSystem(hardwareMap, true);
    }

    public IntakeSystem(HardwareMap hardwareMap, boolean using){
        intake = hardwareMap.get(DcMotorEx.class,"Intake"); //Ex0
    }

    public void cuspirPixel(){
        intake.setPower(-0.7);
        sleep(500);
        intake.setPower(0);
        sleep(200);
    }

    public void stopIntake(){
        intake.setPower(0);
    }

    public void startIntake(){
        intake.setPower(0.65);
    }

    public void reverseIntake(){
        intake.setPower(-1);
    }

    public double getIntakePower(){
        return intake.getPower();
    }

    public double getIntakeCurrent(){
        return intake.getCurrent(CurrentUnit.AMPS);
    }

    private void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
