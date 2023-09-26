#ifndef _KALMAN_FILTER_H
#define _KALMAN_FILTER_H

class kalmanFilter {
private:
    double x_hat;
    double P = 1.0;
    double A = 1.0;
    double H = 1.0;
    double Q;
    double R;
public:
    kalmanFilter(double initial_position, double initial_Q, double initial_R) {
        x_hat = initial_position;
        Q = initial_Q;
        R = initial_R;
    }
    double update(double z) {
        // 预测步骤
        double x_hat_minus = A * x_hat;
        double P_minus = A * A * P + Q;

        // 更新步骤
        double K = P_minus * H / (H * H * P_minus + R);
        x_hat = x_hat_minus + K * (z - H * x_hat_minus);
        P = (1 - K * H) * P_minus;

        return x_hat;
    }
};

#endif