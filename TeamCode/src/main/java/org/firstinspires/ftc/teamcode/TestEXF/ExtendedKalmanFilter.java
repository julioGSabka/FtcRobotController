package org.firstinspires.ftc.teamcode.TestEXF;

public class ExtendedKalmanFilter {

    double WHEEL_RADIUS = 0;
    double W_dis;
    double D_dis;
    double L;
    double l;

    double deltaT;


    double[][] state = new double[6][1];
    double[][] vels = new double[4][1];
    double[][] previous_state = new double[6][1];
    double[][] previous_vels = new double[4][1];
    double[][] measurement = new double[6][1];
    double[][] previous_measurement = new double[6][1];

    double[][] Qvec = { {0.002*deltaT,  0.002*deltaT, 0.002*deltaT, 0.45, 0.45, 0.45} };
    double[][] Rvec = { {0.002*deltaT,  0.002*deltaT, 0.002*deltaT, 0.45, 0.45, 0.45} };


    private void initializeState() {
        // Assuming you have constants defined for WHEEL_RADIUS, L, and l
        state = MatrixSum((stateTransition(previous_state, vels)), diag(Qvec));
        measurement = MatrixSum(stateToMeasurement(state), diag(Rvec));
    }


    public double[][] diag(double[][] matrix){
        double [][] matriz = new double[matrix[0].length][matrix[0].length];
        for(int i=0; i<matrix[0].length; i++){
            matriz[i][i] = matrix[0][i];
        }
        return matriz;
    }

    public double[][] MatrixSum(double[][] Mtx1, double[][] Mtx2) {
        double[][] finalMatrix = new double[Mtx1.length][Mtx2.length];

        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                finalMatrix[i][j] = Mtx1[i][j] + Mtx2[i][j];
            }
        }

        return finalMatrix;
    }

    public double[][] MatrixSubtract(double[][] Mtx1, double[][] Mtx2) {
        double[][] finalMatrix = new double[Mtx1.length][Mtx2.length];

        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                finalMatrix[i][j] = Mtx1[i][j] - Mtx2[i][j];
            }
        }

        return finalMatrix;
    }



    public double[][] stateTransitionJacobian(double[][] state, double[][] vels){
        double[][] A = { { 1, 0, deltaT *((-state[3][0]*Math.sin(state[2][0])) -(state[4][0]*Math.cos(state[2][0]))), deltaT * Math.cos(state[2][0]), -deltaT * Math.sin(state[2][0]), 0 },
                { 0, 1, deltaT *((state[3][0]*Math.cos(state[2][0])) -(state[4][0]*Math.sin(state[2][0]))), deltaT * Math.sin(state[2][0]), deltaT * Math.cos(state[2][0]), 0 },
                { 0, 0, 1, 0, 0, deltaT },
                { 0, 0, 0, 0, 0, 0 },
                { 0, 0, 0, 0, 0, 0 },
                { 0, 0, 0, 0, 0, 0 } };

        return A;
    }

    public double[][] stateMeasurementJacobian(double[][] state, double[][] vels){
        double[][] H = { { 1, 0, 0, 0, 0, 0 },
                { 0, 1, 0, 0, 0, 0 },
                { 0, 0, 1, 0, 0, 0 },
                { 0, 0, 0, 1, 0, -W_dis },
                { 0, 0, 0, 1, 0, W_dis },
                { 0, 0, 0, 0, 1, -D_dis } };

        return H;
    }


    public double[][] stateToMeasurement(double[][] state){

        //Row x Collum
        double[][] A = new double[6][1];
        A [0][0] = state[0][0];
        A [1][0] = state[1][0];
        A [2][0] = state[2][0];
        A [3][0] = state[3][0];
        A [4][0] = state[3][0];
        A [5][0] = state[4][0];

        return A;
    }


    public double[][] stateTransition(double[][] state, double[][] vels){

        double[][] transformed = new double[6][1];

        transformed [0][0] = state[0][0] + deltaT * (state[3][0] * Math.cos(state[2][0]) + state[4][0] * Math.sin(state[2][0]));
        transformed [1][0] = state[1][0] + deltaT * (state[3][0] * Math.sin(state[2][0]) + state[4][0] * Math.cos(state[2][0]));
        transformed [2][0] = state[2][0] + deltaT * state[5][0];
        transformed [3][0] = (WHEEL_RADIUS * 0.25) * (vels[0][0] + vels[1][0] + vels[2][0] + vels[3][0]);
        transformed [4][0] = (WHEEL_RADIUS * 0.25) * (vels[0][0] - vels[1][0] - vels[2][0] + vels[3][0]);
        transformed [5][0] = ((WHEEL_RADIUS) * (0.25*(L + l))) * (vels[0][0] + vels[1][0] - vels[2][0] + vels[3][0]);

        return transformed;
    }

}
