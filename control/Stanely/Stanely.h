/**
 * @brief 前轮反馈控制算法
 * @author ZLLee
 * @date 2024.6.14
*/

#ifndef PLANNING_CONTROL_AUTO_STANELY_H
#define PLANNING_CONTROL_AUTO_STANELY_H
#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
using namespace std;

#define  PI 3.1414926

class Stanely{
private:
    double k;
public:
    Stanely(double k);
    double calTargetIndex(vector<double>robot_state, 
    vector<vector<double>>refer_path);
    double normalizeAngle(double angle);
    vector<double> stanelyControl(vector<double> robot_state,vector<vector<double>> refer_path);
};

#endif