package org.firstinspires.ftc.teamcode.vision.mapper;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;
import java.util.List;

public class KalmanPose {
    KalmanFilter xFilter;
    KalmanFilter yFilter;

    double x = 0;
    double y = 0;

    double headingAdd = 0;

    Rotation2d finalHeading;

    Pose2d correctPose;
    public KalmanPose(Pose2d startpose){
        this.correctPose = startpose;
        xFilter = new KalmanFilter(0.1, 0.4);
        yFilter = new KalmanFilter(0.1, 0.4);
    }
    public void updateFilter(Pose2d vel, List<Pose2d> measurePoses, double heading) {
        heading = headingAdd + heading;

        for (Pose2d pose : measurePoses) {
            x = xFilter.updateFilter(vel.getX()*Math.cos(heading) + vel.getY()*Math.sin(heading), correctPose.getX(), pose.getX());
            y = yFilter.updateFilter(vel.getY()*Math.cos(heading) + vel.getX()*Math.sin(heading), correctPose.getY(), pose.getY());

            correctPose = new Pose2d(x, y, new Rotation2d(heading));
        }
    }

    //isso aq n é do kalman mas a classe é minha e eu faço oq eu quero (até a build dar erro)
    //não é uma fase mãe! Nescau Radical :sunglasses:
    public void addDelta(Translation2d deltaPose, double yaw){
        this.correctPose = new Pose2d(
                this.correctPose.getX() + deltaPose.getX(),
                this.correctPose.getY() + deltaPose.getY(),
                new Rotation2d(yaw + headingAdd));
    }

    public Pose2d getPose(){
        return this.correctPose;
    }

    public void setPose(Pose2d pose){
        this.correctPose = pose;
        this.headingAdd = pose.getHeading();
    }

}
