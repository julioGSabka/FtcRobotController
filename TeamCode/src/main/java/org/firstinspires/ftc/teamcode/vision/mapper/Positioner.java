package org.firstinspires.ftc.teamcode.vision.mapper;


import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;


public class Positioner {
    public static Pose2d tagTheoreticalPose(AprilTagDetection tag){
        return new Pose2d(
                new Translation2d(tag.metadata.fieldPosition.get(0), tag.metadata.fieldPosition.get(1)),
                // pegarmos a rotação EXTRINSICA, ou seja, rotação com relação ao espaço de coordenadas en não do objeto.
                // Rotação em Z ou seja o yaw
                new Rotation2d(Math.toRadians(tag.metadata.fieldPosition.get(3)))
        );
    }

    public static Pose2d tagPose(AprilTagDetection tag){
        return new Pose2d(
                new Translation2d(tag.ftcPose.y, tag.ftcPose.x),
                new Rotation2d(-Math.toRadians(tag.ftcPose.yaw+180))
        );
    }

    public static Pose2d tagToCamPose(AprilTagDetection tag){
        double x, y, Orientation;

        // daq em diante é tudo coordenada do roadrunner

        x = tag.metadata.fieldPosition.get(0) + (tag.ftcPose.y *  Math.cos(Math.toRadians(tag.metadata.fieldPosition.get(3)))) +
                (tag.ftcPose.x * Math.sin(Math.toRadians(tag.metadata.fieldPosition.get(3))));
        y = tag.metadata.fieldPosition.get(1) + (tag.ftcPose.x * -Math.cos(Math.toRadians(tag.metadata.fieldPosition.get(3)))) +
                (tag.ftcPose.x * Math.sin(Math.toRadians(tag.metadata.fieldPosition.get(3))));

        Orientation = Math.toRadians(180 + tag.metadata.fieldPosition.get(3) - tag.ftcPose.yaw);


        double theta = Math.toRadians(-tag.ftcPose.yaw);

        double h = tag.metadata.fieldPosition.get(0);
        double k = tag.metadata.fieldPosition.get(1);

        double xRotacionado = (((x - h) * Math.cos(theta)) - ((y - k) * Math.sin(theta))) + h;
        double yRotacionado = (((x - h) * Math.sin(theta)) + ((y - k) * Math.cos(theta))) + k;


        Pose2d camPose = new Pose2d(new Translation2d(xRotacionado, yRotacionado), new Rotation2d(Orientation));

        return camPose;
    }

    public static Pose2d getRobotPose(AprilTagDetection tag, Transform2d cameraTransform){
        // uses tag pose and camera transform to get robot transform
        // cameraTransform should be from robot origin to camera
        return tagToCamPose(tag) //gets the camera pose based on tag
                .transformBy(cameraTransform.inverse()); //transforms it from camera to robot
    }
}
