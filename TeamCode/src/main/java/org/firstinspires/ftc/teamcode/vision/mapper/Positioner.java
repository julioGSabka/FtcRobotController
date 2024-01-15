package org.firstinspires.ftc.teamcode.vision.mapper;


import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

import org.apache.commons.math3.complex.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
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
        //cam position relative to tag on Origin
        Pose2d cam_base_tag_pose = tagPose(tag);
        //transforms arena coordinate to tag coordinate
        Transform2d arena_to_tag = new Transform2d(
                new Translation2d(tag.metadata.fieldPosition.get(0), tag.metadata.fieldPosition.get(1)),
                // pegarmos a rotação EXTRINSICA, ou seja, rotação com relação ao espaço de coordenadas en não do objeto.
                // Rotação em Z ou seja o yaw
                new Rotation2d(Math.toRadians(tag.metadata.fieldPosition.get(3)))
        );

        // gets the inverse, this case transforming tag coordinate to arena coordinate
        Transform2d tag_to_arena = arena_to_tag.inverse();

        Pose2d camPose = cam_base_tag_pose.transformBy(tag_to_arena);

        return camPose;
    }

    public static Pose2d getRobotPose(AprilTagDetection tag, Transform2d cameraTransform){
        // uses tag pose and camera transform to get robot transform
        // cameraTransform should be from robot origin to camera
        return tagToCamPose(tag) //gets the camera pose based on tag
                .transformBy(cameraTransform.inverse()); //transforms it from camera to robot
    }
}
