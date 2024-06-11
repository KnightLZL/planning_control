/**
 * @author ZLLee
 * 贝塞尔曲线
*/

#ifndef PLANNING_CONTROL_AUTO_BEZIER_H
#define PLANNING_CONTROL_AUTO_BEZIER_H

#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <time.h>
#include <algorithm>

using namespace std;
using namespace Eigen;

double factorial(int n);

Vector2d bezierCommon(vector<Vector2d> Ps, double t);

// vector2d bezierRecursion(vector<Vector2d> Ps, double t);

#endif