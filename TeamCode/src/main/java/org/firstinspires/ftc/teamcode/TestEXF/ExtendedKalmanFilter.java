package org.firstinspires.ftc.teamcode.TestEXF;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.DecompositionSolver;
import org.apache.commons.math3.linear.LUDecomposition;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;

import java.util.ArrayList;
import java.util.List;


public class ExtendedKalmanFilter {
    /*
    double[][] P = new double[6][6];
    double[][] states = new double[1][6];
    double[][] previous_state = new double[1][6];

    double[][] A = new double[6][6];

    boolean firstUpdate = true;
    boolean firstPredictUpdate = true;

    double[][] predictions = initialState;
    double[][] measurments = initialState;

    double[][] rawMeasurements = { {0, 0, 0, 0, 0, 0} };
    double[][] predictMeasurements = { {0, 0, 0, 0, 0, 0} };


    public void reset(){
        this.P = new double[6][6];
        this.states = new double[1][6];
        this.previous_state = new double[1][6];
        this.A = new double[6][6];
        boolean firstUpdate = true;
        boolean firstPredictUpdate = true;

        this.predictions = initialState;
        this.measurments = initialState;
        this.rawMeasurements = new double[][]{{0, 0, 0, 0, 0, 0}};
        this.predictMeasurements = new double[][]{{0, 0, 0, 0, 0, 0}};

    }

    public double[][] stateToMeasurement(double[][] state){
        //Row x Collum
        double[][] A = new double[6][1];
        A [0][0] = state[0][0];
        A [1][0] = state[1][0];
        A [2][0] = state[2][0];
        A [3][0] = state[3][0];
        A [4][0] = state[4][0];
        A [5][0] = state[5][0];

        return A;
    }

    public List<double[][]> step(double[][] u, double[][] measurment){

        List<Double> returns = new ArrayList<>();

        double prev_state = getPrevState();
        List<Double> predictionANDq = predict(u, prev_state);

        List<Double> bestImage = findBestImage(prediction);
        if (detected){

        }
        double[][] R ={{(1E-3)*(d*d) + 1E-3, 0, 0, 0, 0, 0},
                       {0, (1E-3)*(d*d) + 1E-3, 0, 0, 0, 0},
                       {0, 0, (1E-3)*(d*d) + 1E-3, 0, 0, 0},
                       {0, 0, 0, 2E-3 * measurement[3][0] * measurement[3][0] + 1E-3, 0, 0},
                       {0, 0, 0, 0, 2E-3 * measurement[4][0] * measurement[4][0] + 1E-3, 0},
                       {0, 0, 0, 0, 0, 2E-3 * measurement[5][0] * measurement[5][0] + 1E-3}};

        if (mode == 1) {
            for ( int i = 3; i < 6; i++){
                R[i][i] = 99999999;
            }
        }

        List<Double> measured_var = new ArrayList<Double>();
        List<Double> predicted_var = new ArrayList<Double>();

        measured_var.add(0,R[3][3] + R[4][4] + R[5][5]);
        predicted_var.add(0, Q[3][3] + Q[4][4] + Q[5][5]);

        double[][] H = {{1.0, 0, 0, 0, 0, 0},
                      {0, 1.0, 0, 0, 0, 0},
                      {0, 0, 1.0, 0, 0, 0},
                      {0, 0, 0, 1.0, 0, -W_dis},
                      {0, 0, 0, 1.0, 0, W_dis},
                      {0, 0, 0, 0, 1.0, -D_dis}};
        A = this.A;
        P = this.P;
        double[][] predicted_measurement = stateToMeasurement(predictionANDq.get(0).T[0]);

        double[][] P_prediction = MatrixSum((multiplyMatrices(multiplyMatrices(A, P), A.T)), Q);

        double[][] K = LinearSolver();

        K = np.linalg.solve((H @ P_prediction @ H.T + R).T, (P_prediction @ H.T).T).T ;
        estimate = prediction + K @ (measurement - predicted_measurement);
        this.P = P_prediction - K @ H @ P_prediction;

        return returns;
    }

    public double getPrevState(){
        if (this.firstUpdate){
            return initial_state;
        } else {
            return this.states[0][-1];
        }
    }

    public static void LinearSolver(double[][] MTX1, double[][] MTX2) {
        // Exemplo de sistema de equações lineares:
        // 2x + y = 5
        // 3x - 2y = -8

        RealMatrix coefficients =
                new Array2DRowRealMatrix(new double[][] { { 2, 1 }, { -1, 7, 6 }, { 4, -3, -5 } },
                        false);
        DecompositionSolver solver = new LUDecomposition(coefficients).getSolver();

        RealVector constants = new ArrayRealVector(new double[] { 5, -2, 1 }, false);
        RealVector solution = solver.solve(constants);
    }

    public double[][] multiplyMatrices(double[][] firstMatrix, double[][] secondMatrix) {
        double[][] result = new double[firstMatrix.length][secondMatrix[0].length];

        for (int row = 0; row < result.length; row++) {
            for (int col = 0; col < result[row].length; col++) {
                result[row][col] = multiplyMatricesCell(firstMatrix, secondMatrix, row, col);
            }
        }

        return result;
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

    /*
    private void initializeState() {
        // Assuming you have constants defined for WHEEL_RADIUS, L, and l
        state = MatrixSum((stateTransition(previous_state, vels)), diag(Qvec));
        measurement = MatrixSum(stateToMeasurement(state), diag(Rvec));
        stateP = stateTransition(state, vels)
    }


    public double[][] diag(double[][] matrix){
        double [][] matriz = new double[matrix[0].length][matrix[0].length];
        for(int i=0; i<matrix[0].length; i++){
            matriz[i][i] = matrix[0][i];
        }
        return matriz;
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
    */

}
