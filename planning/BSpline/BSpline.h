/**
 * B样条曲线
*/

#ifndef PLANNING_CONTROL_AUTO_BSPLINE_H
#define PLANNING_CONTROL_AUTO_BSPLINE_H
#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <algorithm>

using namespace std;
using namespace Eigen;

double baseFunction(int i, int k, double u, vector<double>node_vector);

vector<double> u_quasi_uniform(int n, int k);

vector<double> u_piecewise_B_Spline(int n, int k);


#endif 