package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.ArrayList;

public class ElevationSystem {

    private DcMotorEx motorElevacaoL = null;
    private DcMotorEx motorElevacaoR = null;
    private ServoImplEx servoElevacaoL = null;
    private ServoImplEx servoElevacaoR = null;

    private int maxSpeed = 2400;
    private double tresholdCurrent = 1.5;
    private int liftTarget = 500;

    private boolean isLifting = false;

    public ElevationSystem(HardwareMap hardwareMap) {
        new ElevationSystem(hardwareMap, true);
    }

    public ElevationSystem(HardwareMap hardwareMap, boolean using){

        motorElevacaoL = hardwareMap.get(DcMotorEx.class, "motorElevacaoL"); //Ex2
        motorElevacaoR = hardwareMap.get(DcMotorEx.class, "motorElevacaoR"); //Ex3
        servoElevacaoL = hardwareMap.get(ServoImplEx.class, "servoElevacaoL"); //Ex2
        servoElevacaoR = hardwareMap.get(ServoImplEx.class, "servoElevacaoR"); //Ex4

        motorElevacaoL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorElevacaoR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        motorElevacaoL.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public void UpGanchos() {
        servoElevacaoL.setPosition(0);
        servoElevacaoR.setPosition(1);
    }

    public void DownGanchos() {
        servoElevacaoL.setPosition(1);
        servoElevacaoR.setPosition(0);
    }
    public void joystickControl(double MtrRight, double MtrLeft) {
        if(!isLifting){
            motorElevacaoR.setPower(MtrRight);
            motorElevacaoL.setPower(MtrLeft);
        }else{
            motorElevacaoR.setVelocity((MtrRight*0.3)+0.7 * maxSpeed);
            motorElevacaoL.setVelocity((MtrLeft*0.3)+0.7 * maxSpeed);
        }

    }

    public void StopMotors(){
        motorElevacaoL.setPower(0);
        motorElevacaoR.setPower(0);
    }

    public void ReverseMotors(){
        motorElevacaoL.setPower(-0.5);
        motorElevacaoR.setPower(-0.5);
    }

    public void TensionCord(){
        motorElevacaoL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorElevacaoR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorElevacaoL.setPower(0.3);
        motorElevacaoR.setPower(0.3);
        boolean aligned = false;
        boolean Laligned = false;
        boolean Raligned = false;
        /*
        TODO: colocar checagem de SE OPMODE ATIVO!!!
         */
        while(!aligned){
            if(motorElevacaoL.getCurrent(CurrentUnit.AMPS)>tresholdCurrent && !Laligned){
                Laligned = true;
                motorElevacaoL.setTargetPosition(motorElevacaoL.getCurrentPosition());
                motorElevacaoL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }
            if(motorElevacaoR.getCurrent(CurrentUnit.AMPS)>tresholdCurrent && !Raligned){
                Raligned = true;
                motorElevacaoR.setTargetPosition(motorElevacaoR.getCurrentPosition());
                motorElevacaoR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }
            if(Laligned && Raligned){
                aligned = true;
            }
        }
    }

    public void liftUpRobot(){
        if(!isLifting) {
            isLifting = true;
            motorElevacaoL.setTargetPosition(motorElevacaoL.getCurrentPosition() + liftTarget);
            motorElevacaoR.setTargetPosition(motorElevacaoR.getCurrentPosition() + liftTarget);
        }
    }


    public ArrayList<Double> getMotorsCurrent() {
        ArrayList<Double> current = new ArrayList<>();
        current.add(motorElevacaoR.getCurrent(CurrentUnit.AMPS));
        current.add(motorElevacaoL.getCurrent(CurrentUnit.AMPS));

        return current;
    }

    public ArrayList<Integer> getMotorsCurrentPosition() {
        ArrayList<Integer> position = new ArrayList<>();
        position.add(motorElevacaoR.getCurrentPosition());
        position.add(motorElevacaoL.getCurrentPosition());

        return position;
    }

    public ArrayList<Integer> getMotorsTargetPosition() {
        ArrayList<Integer> position = new ArrayList<>();
        position.add(motorElevacaoR.getTargetPosition());
        position.add(motorElevacaoL.getTargetPosition());

        return position;
    }

    public void onStopBehavoiur(){
        motorElevacaoL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorElevacaoL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorElevacaoL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorElevacaoR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorElevacaoL.setPower(0);
        motorElevacaoR.setPower(0);
    }

}
