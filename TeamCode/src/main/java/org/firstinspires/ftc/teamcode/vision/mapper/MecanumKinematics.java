package org.firstinspires.ftc.teamcode.vision.mapper;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

public class MecanumKinematics {

    public static Pose2d wheelToVel(double vFrontLeft, double vFrontRight, double vBackLeft, double vBackRight, double lengthX, double lengthY){
        return new Pose2d(
                new Translation2d(
                        (vFrontLeft + vFrontRight + vBackLeft + vBackRight)*0.25,
                        (-vFrontLeft + vFrontRight + vBackLeft - vBackRight)*0.25
                ),
                new Rotation2d(
                        (-vFrontLeft + vFrontRight - vBackLeft + vBackRight)/(lengthX + lengthY)
                )
        );
    }

    double lastPFrontLeft = 0;
    double lastPFrontRight = 0;
    double lastPBackLeft = 0;
    double lastPBackRight = 0;


    public Pose2d mecanumDeltaPose(double pFrontLeft, double pFrontRight, double pBackLeft, double pBackRight, double lengthX, double lengthY){
        Pose2d delPose = wheelToVel(
                pFrontLeft - this.lastPFrontLeft,
                pFrontRight - this.lastPFrontRight,
                pBackLeft - this.lastPBackLeft,
                pBackRight - this.lastPBackRight,
                lengthX,
                lengthY
                );

        this.lastPFrontLeft = pFrontLeft;
        this.lastPFrontRight = pFrontRight;
        this.lastPBackLeft = pBackLeft;
        this.lastPBackRight = pBackRight;
        return delPose;
    }
}
