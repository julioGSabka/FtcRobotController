package org.firstinspires.ftc.teamcode.vision.mapper;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;
import java.util.List;

public class KalmanPose {
    KalmanFilter xFilter;
    KalmanFilter yFilter;
    KalmanFilter wFilter;

    KalmanFilter qwFilter;
    KalmanFilter qxFilter;
    KalmanFilter qyFilter;
    KalmanFilter qzFilter;

    double x = 0;
    double y = 0;
    double w = 0;

    double qw = 0;
    double qx = 0;
    double qy = 0;
    double qz = 0;

    Rotation2d finalHeading;

    Pose2d correctPose;
    public KalmanPose(Pose2d startpose){
        this.correctPose = startpose;
    }
    public void updateFilter(Pose2d vel, List<Pose2d> measurePoses, double heading) {
        xFilter = new KalmanFilter(0.1, 0.4);
        yFilter = new KalmanFilter(0.1, 0.4);
        wFilter = new KalmanFilter(0.1, 0.4);

        for (Pose2d pose : measurePoses) {
            x = xFilter.updateFilter(vel.getX()*Math.cos(heading) + vel.getY()*Math.sin(heading), correctPose.getX(), pose.getX());
            y = yFilter.updateFilter(vel.getY()*Math.cos(heading) + vel.getX()*Math.sin(heading), correctPose.getY(), pose.getY());



            /*
            List<Double> quatPose = euler_to_quaternion(0,0, pose.getHeading());
            List<Double> quatVel = euler_to_quaternion(0,0, vel.getHeading());
            List<Double> quatFilterPose = euler_to_quaternion(0,0, vel.getHeading());

            qw = qwFilter.updateFilter(quatVel.get(0), quatFilterPose.get(0), quatPose.get(0));
            qx = qxFilter.updateFilter(quatVel.get(1), quatFilterPose.get(1), quatPose.get(1));
            qy = qyFilter.updateFilter(quatVel.get(2), quatFilterPose.get(2), quatPose.get(2));
            qz = qzFilter.updateFilter(quatVel.get(3), quatFilterPose.get(3), quatPose.get(3));

            List<Double> Euler = quaternion_to_euler(qw, qx, qy, qz);
             */

            correctPose = new Pose2d(x, y, new Rotation2d(heading));
        }
    }

    public static List<Double> euler_to_quaternion(double roll, double pitch, double yaw){
        double q_w = Math.cos(roll / 2) * Math.cos(pitch / 2) * Math.cos(yaw / 2) + Math.sin(roll / 2) * Math.sin(pitch / 2) * Math.sin(yaw / 2);
        double q_x = Math.sin(roll / 2) * Math.cos(pitch / 2) * Math.cos(yaw / 2) - Math.cos(roll / 2) * Math.sin(pitch / 2) * Math.sin(yaw / 2);
        double q_y = Math.cos(roll / 2) * Math.sin(pitch / 2) * Math.cos(yaw / 2) + Math.sin(roll / 2) * Math.cos(pitch / 2) * Math.sin(yaw / 2);
        double q_z = Math.cos(roll / 2) * Math.cos(pitch / 2) * Math.sin(yaw / 2) - Math.sin(roll / 2) * Math.sin(pitch / 2) * Math.cos(yaw / 2);

        List<Double> quaternion = new ArrayList<>();
        quaternion.add(q_w);
        quaternion.add(q_x);
        quaternion.add(q_y);
        quaternion.add(q_z);

        return quaternion;
    }

    public static List<Double> quaternion_to_euler(double q_w, double q_x, double q_y, double q_z){
        double phi = Math.toDegrees(Math.atan2(2 * (q_w * q_x + q_y * q_z), 1 - 2 * (q_x * q_x + q_y * q_y)));
        double theta = Math.toDegrees(Math.asin(2 * (q_w * q_y - q_z * q_x)));
        double omega = Math.toDegrees(Math.atan2(2 * (q_w * q_z + q_x * q_y), 1 - 2 * (q_y * q_y + q_z * q_z)));

        List<Double> euler = new ArrayList<>();
        euler.add(phi);
        euler.add(theta);
        euler.add(omega);

        return euler;
    }

    //isso aq ne é do kalman mas a classe é minha e eu faço oq eu quero (até a build dar erro)
    //não é uma fase mãe!
    public void addDelta(Pose2d deltaPose){
        this.correctPose = new Pose2d(
                this.correctPose.getX() + deltaPose.getX(),
                this.correctPose.getY() + deltaPose.getY(),
                this.correctPose.getRotation().plus(deltaPose.getRotation()));
    }

    public Pose2d getPose(){
        return this.correctPose;
    }

}
