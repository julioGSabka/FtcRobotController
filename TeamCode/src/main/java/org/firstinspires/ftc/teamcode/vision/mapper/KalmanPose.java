package org.firstinspires.ftc.teamcode.vision.mapper;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import java.util.List;

public class KalmanPose {
    KalmanFilter xFilter;
    KalmanFilter yFilter;
    KalmanFilter wFilter;

    double x = 0;
    double y = 0;
    Rotation2d w;

    Pose2d correctPose;

    Pose2d updateFilter(Pose2d vel, Pose2d statePose, List<Pose2d> measurePoses) {

        xFilter = new KalmanFilter(0.1, 0.4);
        yFilter = new KalmanFilter(0.1, 0.4);
        wFilter = new KalmanFilter(0.1, 0.4);

        Pose2d filterPose = statePose;

        for (Pose2d pose : measurePoses) {
            x = xFilter.updateFilter(vel.getX(), filterPose.getX(), pose.getX());
            y = yFilter.updateFilter(vel.getY(), filterPose.getY(), pose.getY());
            w = new Rotation2d(wFilter.updateFilter(vel.getHeading(), filterPose.getHeading(), pose.getHeading()));

            filterPose = new Pose2d(x, y, w);
        }

        correctPose = filterPose;

        return correctPose;
    }

}
