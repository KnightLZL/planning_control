/**
 * 车辆运动学模型
*/

#ifndef PLANNING_CONTROL_KINEMATICMODEL_H
#define PLANNING_CONTROL_KINEMATICMODEL_H
#include <iostream>
#include <vector>
#include <cmath>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

class KinematicModel {
public:
    double x, y, psi, v, L, dt;
public:
    //默认构造函数
    KinematicModel();

    /**
     * 车辆运动学模型构造
     * @param x 位置x
     * @param y 位置y
     * @param psi 偏航角
     * @param v 速度
     * @param l 轴距
     * @param dt 采样时间
     */
    KinematicModel(double x, double y, double psi, double v, double l, double dt) {
        this->x = x;
        this->y = y;
        this->psi = psi;
        this->v = v;
        this->L = l;
        this->dt = dt;
    }

    /** 更新车辆状态
     * 控制量为转向角delta_f和加速度a
     * @param accel 加速度
     * @param delta_f 转向角
    */
   void updateState(double accel, double delta_f){
        x = x + v*cos(psi)*dt;
        y = y + v*sin(psi)*dt;
        psi = psi + v/L * tan(delta_f)*dt;
        v = v + accel*dt;
   }

    /**
        * 获取车辆状态
        * @param return
    */
    vector<double> getState(){
    return {x, y, psi, v};
    }

    /** 离散车辆状态方程
     * 离散状态空间方程   以后轴为中心的车辆中心的单车运动学模型离散化
     * @param ref_delta  名义控制输入
     * @param ref_yaw  名义航向角
     * @return 
    */
    vector<MatrixXd> stateSpace(double ref_delta, double ref_yaw) {
        MatrixXd A(3, 3), B(3, 2);
        A<<1.0, 0.0, -v*dt*sin(ref_yaw),
           0.0, 0.0, v*dt*cos(ref_yaw),
           0.0, 0.0, 1;
        B<<dt*cos(ref_yaw), 0.0,
           dt*sin(ref_yaw), 0.0,
           dt*tan(ref_delta)/L, v*dt/L/pow(cos(ref_delta), 2);
        return {A, B};
    }

};

#endif