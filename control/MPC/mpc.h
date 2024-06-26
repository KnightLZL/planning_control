/**
 * @date 2024.6.26
*/
#ifndef PLANNING_CONTROL_AUTO_MPC_H
#define PLANNING_CONTROL_AUTO_MPC_H

#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include "../utils/KinematicModel.hpp"
#include "OsqpEigen/OsqpEigen.h"
#include <osqp/osqp.h>
#include "../../matplotlibcpp.h"

// #include <iostream>
// #include <vector>
// #include <Eigen/Dense>
// #include "../utils/KinematicModel.hpp"
// #include "/home/lee/osqp-eigen/include/OsqpEigen/OsqpEigen.h"
// #include "/home/lee/osqp/include/osqp.h"
// #include "../../matplotlibcpp.h"

namespace plt = matplotlibcpp;
using namespace std;
using namespace Eigen;
using namespace OsqpEigen;

namespace{
    const double MAX_STEER = M_PI / 4.0; // maximum steering angle [rad]
    const double MAX_VEL = 2.0; // maximum velocity [m/s]
}

class MPCControl {
private:
    int NX,NU,T;
    MatrixXd R = MatrixXd::Identity(NU, NU);  // input cost matrix
    MatrixXd Rd = MatrixXd::Identity(NU, NU); // input difference cost matrix
    MatrixXd Q = MatrixXd::Identity(NX, NX);  // state cost matrix
    MatrixXd Qf = Q; // state final matrix

public:
    MPCControl(int nx, int nu, int t);
    vector<double> linearMPCControl(MatrixXd xref, Vector3d x0, MatrixXd ref_delta, KinematicModel ugv);



};


#endif 