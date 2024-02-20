package org.firstinspires.ftc.teamcode.vision.mapper;

public class KalmanFilter {
    double x; // your initial state
    double Q = 0.1; // your model covariance
    double R = 0.4; // your sensor covariance
    double p; // your initial covariance guess
    double K; // your initial Kalman gain guess

    double x_previous = x;
    double p_previous = p;
    double u;
    double z;

    KalmanFilter(double Q, double R){
        this.Q = Q;
        this.R = R;
    }

    double updateFilter(double velocity, double statePose, double measurePose){

        x = statePose;
        u = velocity; // Ex: change in position from odometry.
        x = x_previous + u;

        p = p_previous + Q;

        K = p/(p + R);

        z = measurePose; // Pose Estimate from April Tag / Distance Sensor

        x = x + K * (z - x);

        p = (1 - K) * p;

        x_previous = x;
        p_previous = p;

        return x;
    }

}
