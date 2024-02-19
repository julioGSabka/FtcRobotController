package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class ArmSystem {

    private Servo garra = null;
    private Servo cotovelo = null;
    private Servo ombro = null;

    public ArmSystem(HardwareMap hardwareMap) {
        new ArmSystem(hardwareMap, true);
    }

    public ArmSystem(HardwareMap hardwareMap, boolean using){

        garra = hardwareMap.get(Servo.class, "garra"); //Ex0
        cotovelo = hardwareMap.get(Servo.class, "cotovelo"); //4
        ombro = hardwareMap.get(Servo.class, "ombro"); //2

    }

    public void UpArm() {
        cotovelo.setPosition(0.8);
        ombro.setPosition(0.95);
        sleep(500);
        cotovelo.setPosition(0.97);
        sleep(500);
        ombro.setPosition(0);
        sleep(500);
        cotovelo.setPosition(0.35);
    }
    //Cotovelo: 1 - Recolhido
    //Ombro: 1 - Abaixado

    public void DownArm() {
        cotovelo.setPosition(0.97);
        sleep(500);
        ombro.setPosition(0.95);
        sleep(750);
        cotovelo.setPosition(0.8);
        sleep(500);
        ombro.setPosition(0.87);
        cotovelo.setPosition(0.69);

    }

    public void closeGarra() {
        garra.setPosition(0);
    }
    public void midlleGarra() {
        garra.setPosition(0.5);
    }
    public void openGarra() {
        garra.setPosition(1);
    }

    public void setGarra(double pos) {
        garra.setPosition(pos);
    }
    public void setCotovelo(double pos) {
        cotovelo.setPosition(pos);
    }
    public void setOmbro(double pos) {
        ombro.setPosition(pos);
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
