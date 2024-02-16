package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class ArmSystem {

    private Servo garra = null;
    private ServoImplEx cotovelo = null;
    private ServoImplEx ombro = null;

    public ArmSystem(HardwareMap hardwareMap) {
        new ArmSystem(hardwareMap, true);
    }

    public ArmSystem(HardwareMap hardwareMap, boolean using){

        garra = hardwareMap.get(Servo.class, "garra"); //Ex0
        cotovelo = hardwareMap.get(ServoImplEx.class, "cotovelo"); //4
        ombro = hardwareMap.get(ServoImplEx.class, "ombroR"); //2

    }

    public void UpArm() {
        ombro.setPosition(0.75);
        sleep(500);
        cotovelo.setPosition(1);
        sleep(1000);
        ombro.setPosition(0.5);
        sleep(500);
        cotovelo.setPosition(0.35);
    }

    public void DownArm() {
        cotovelo.setPosition(1);
        sleep(500);
        ombro.setPosition(0.75);
        sleep(500);
        cotovelo.setPosition(0.69);
        sleep(500);
        ombro.setPosition(0.87);

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

    public double getOmbroPos() {
        return ombro.getPosition();
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
