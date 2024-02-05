package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class ArmSystem {

    private Servo garra = null;
    private ServoImplEx cotovelo = null;
    private ServoImplEx ombroR = null;
    private ServoImplEx ombroL = null;

    public ArmSystem(HardwareMap hardwareMap) {
        new ArmSystem(hardwareMap, true);
    }

    public ArmSystem(HardwareMap hardwareMap, boolean using){

        garra = hardwareMap.get(Servo.class, "garra"); //Ex0
        cotovelo = hardwareMap.get(ServoImplEx.class, "cotovelo"); //4
        ombroR = hardwareMap.get(ServoImplEx.class, "ombroR"); //2
        ombroL = hardwareMap.get(ServoImplEx.class, "ombroL"); //0

    }

    public void UpArm() {
        cotovelo.setPosition(1);
        sleep(1000);
        ombroL.setPosition(1);
        ombroR.setPosition(0);
        sleep(500);
        cotovelo.setPosition(0.35);
    }

    public void DownArm() {
        cotovelo.setPosition(1);
        sleep(500);
        ombroL.setPosition(0.15);
        ombroR.setPosition(0.85);
        sleep(500);
        ombroL.setPosition(0.09);
        ombroR.setPosition(0.91);
        sleep(1000);
        cotovelo.setPosition(0.724);
        sleep(1000);
    }

    public void closeGarra() {
        garra.setPosition(1);
    }
    public void midlleGarra() {
        garra.setPosition(0.7);
    }
    public void openGarra() {
        garra.setPosition(0);
    }

    public void setGarra(double pos) {
        garra.setPosition(pos);
    }

    public double getOmbroRPos() {
        return ombroR.getPosition();
    }

    public double getOmbroLPos() {
        return ombroL.getPosition();
    }

    public double getCotoveloPos() {
        return cotovelo.getPosition();
    }

    public double getGarraPos() {
        return garra.getPosition();
    }

    private void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

}
