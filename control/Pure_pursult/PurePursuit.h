/**
 * @author ZLLee
 * @brief 纯跟踪算法
*/

#ifndef PLANNING_CONTROL_AUTO_PUREPURSUIT_H
#define PLANNING_CONTROL_AUTO_PUREPURSUIT_H
#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>

using namespace std;
class PurePursuit {
public:
    double calTargetIndex(vector<double> robot_state, vector<vector<double>>refer_path, double l_d);

    double purePursuitControl(vector<double>robot_sate, vector<double>current_ref_point, double l_d, double psi, double L);
};

#endif