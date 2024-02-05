package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.ArrayList;

public class ElevationSystem {

    private DcMotorEx motorElevacaoL = null;
    private DcMotorEx motorElevacaoR = null;
    private ServoImplEx servoElevacaoL = null;
    private ServoImplEx servoElevacaoR = null;

    public ElevationSystem(HardwareMap hardwareMap) {
        new ElevationSystem(hardwareMap, true);
    }

    public ElevationSystem(HardwareMap hardwareMap, boolean using){

        motorElevacaoL = hardwareMap.get(DcMotorEx.class, "motorElevacaoL"); //Ex2
        motorElevacaoR = hardwareMap.get(DcMotorEx.class, "motorElevacaoR"); //Ex3
        servoElevacaoL = hardwareMap.get(ServoImplEx.class, "servoElevacaoL"); //Ex2
        servoElevacaoR = hardwareMap.get(ServoImplEx.class, "servoElevacaoR"); //Ex4

        motorElevacaoL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorElevacaoR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void UpGanchos() {
        servoElevacaoL.setPosition(1);
        servoElevacaoR.setPosition(0);
    }

    public void DownGanchos() {
        servoElevacaoL.setPosition(0);
        servoElevacaoR.setPosition(1);
    }

    public void setMotorsPower(double MtrRight, double MtrLeft) {
        motorElevacaoR.setPower(MtrRight);
        motorElevacaoL.setPower(MtrLeft);
    }

    public ArrayList<Double> getMotorsCurrent() {
        ArrayList<Double> current = new ArrayList<>();
        current.add(motorElevacaoR.getCurrent(CurrentUnit.AMPS));
        current.add(motorElevacaoL.getCurrent(CurrentUnit.AMPS));

        return current;
    }

}
