/**
 * @brief 基于运动学模型的LQR控制
 * @date 2024.6.15
*/

#ifndef PLANNING_CONTROL_AUTO_LQR_H
#define PLANNING_CONTROL_AUTO_LQR_H

#define EPS 1.0e-4
#include <iostream>
#include <vector>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

class LQRControl{
private:
    int N;

public:
    LQRControl(int n);

    MatrixXd calRicatti(MatrixXd A, MatrixXd B, MatrixXd Q, MatrixXd R);

    double lqrControl(vector<double> robot_state, vector<vector<double>> refer_path, double s0, MatrixXd A, MatrixXd B, MatrixXd Q, MatrixXd R);
};

#endif
