package org.firstinspires.ftc.teamcode.vision.mapper;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

public class MecanumKinematics {

    static Pose2d wheelToVel(float vFrontLeft, float vFrontRight, float vBackLeft, float vBackRight, float lengthX, float lengthY){
        return new Pose2d(
                new Translation2d(
                        vFrontLeft + vFrontRight + vBackLeft + vBackRight,
                        vFrontLeft - vFrontRight - vBackLeft + vBackRight
                ),
                new Rotation2d(
                        -vFrontLeft/(lengthX + lengthY) +
                                vFrontRight/(lengthX + lengthY) +
                                -vBackLeft/(lengthX + lengthY) +
                                vBackRight/(lengthX + lengthY)
                )
        );
    }
}
