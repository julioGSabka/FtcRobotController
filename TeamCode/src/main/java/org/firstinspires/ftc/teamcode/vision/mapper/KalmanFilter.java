package org.firstinspires.ftc.teamcode.vision.mapper;

import com.arcrobotics.ftclib.geometry.Pose2d;

import org.apache.commons.math3.linear.RealMatrix;

import java.util.List;

public class KalmanFilter {
    RealMatrix x; // your initial state
    double Q = 0.1; // your model covariance
    double R = 0.4; // your sensor covariance
    RealMatrix p; // your initial covariance guess
    RealMatrix K; // your initial Kalman gain guess

    RealMatrix x_previous = x;
    RealMatrix p_previous = p;
    RealMatrix u;
    RealMatrix z;

    KalmanFilter(double Q, double R){
        this.Q = Q;
        this.R = R;
    }

    Pose2d updateFilter(Pose2d velocity, List<Pose2d> tagEstimates){
        //TODO: converter pose2d para real matrix

        u = getInput(); // Ex: change in position from odometry.


        x = x_previous.add(u);

        p = p_previous.scalarAdd(Q);

        //TODO: fazer divisão para cada elemento de la matrizzz
        K = p/(p.scalarAdd(R));

        //TODO: converter pose2d para real matrix
        z = getSecondSensor(); // Pose Estimate from April Tag / Distance Sensor

        x = x.add(K.multiply(z.subtract(x)));

        p = (K.scalarMultiply(-1).scalarAdd(1)).multiply(p);

        x_previous = x;
        p_previous = p;

        return new Pose2d();
    }
}
