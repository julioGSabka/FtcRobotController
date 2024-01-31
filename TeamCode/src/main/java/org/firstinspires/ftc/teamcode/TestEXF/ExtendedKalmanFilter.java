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


    private void initializeState() {
        // Assuming you have constants defined for WHEEL_RADIUS, L, and l
        state = stateTransition(previous_state, previous_vels);
        measurement = stateToMeasurement(state) + measurement;

    }
    /*
    public RealMatrix stateTransitionJacobian(RealMatrix state, RealMatrix vels){
        double[][] A = { { 1, 0, deltaT *((-state.getEntry(3,0)*Math.sin(state.getEntry(2,0))) -(state.getEntry(4,0)*Math.cos(state.getEntry(2,0)))), deltaT * Math.cos(state.getEntry(2,0)), -deltaT * Math.sin(state.getEntry(2,0)), 0 },
                { 0, 1, deltaT *((state.get(3)*Math.cos(state.get(2))) -(state.get(4)*Math.sin(state.get(2)))), deltaT * Math.sin(state.get(2)), deltaT * Math.cos(state.get(2)), 0 },
                { 0, 0, 1, 0, 0, deltaT },
                { 0, 0, 0, 0, 0, 0 },
                { 0, 0, 0, 0, 0, 0 },
                { 0, 0, 0, 0, 0, 0 } };

        return A;
    }

    public double[][] stateMeasurementJacobian(List<Double> state, List<Double> vels){
        double[][] H = { { 1, 0, 0, 0, 0, 0 },
                         { 0, 1, 0, 0, 0, 0 },
                         { 0, 0, 1, 0, 0, 0 },
                         { 0, 0, 0, 1, 0, -W_dis },
                         { 0, 0, 0, 1, 0, W_dis },
                         { 0, 0, 0, 0, 1, -D_dis } };

        return H;
    }

    public List<Double> stateToMeasurement(List<Double> state){
        double theta = state.get(2);
        double xVel = state.get(3);
        double yVel = state.get(4);
        double thetaVel = state.get(5);

        double a = yVel - D_dis * thetaVel;
        double R = xVel + W_dis * thetaVel;
        double L = xVel - W_dis * thetaVel;

        List<Double> transformed = new ArrayList<>();
        transformed.add(state.get(0));
        transformed.add(state.get(1));
        transformed.add(state.get(2));
        transformed.add(L);
        transformed.add(R);
        transformed.add(a);

        return transformed;
    }
    */

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
