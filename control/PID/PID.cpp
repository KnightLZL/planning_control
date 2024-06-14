/**
 * @date 2024.6.14
*/

#include "PID.h"


PID_controller::PID_controller(double kp, double ki, double kd, double target, double upper, double lower) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    this->target = target;
    this->upper = upper;
    this->lower = lower;
}

/**
 * 设置目标
 * @param target
*/
void PID_controller::setTarget(double target){
    PID_controller::target = target;
}

/**
 * 设置控制器参数
 * @param kp, ki, kd
*/
void PID_controller::setK(double kp, double ki, double kd){
    PID_controller::kd = kd;
    PID_controller::ki = ki;
    PID_controller::kp = kp;
}

/**
 * 设置控制量边界
*/
void PID_controller::setBound(double upper, double lower){
    this->upper = upper;
    this->lower = lower;
}

/**
 * 计算控制输出
*/
double PID_controller::calOutput(double state) {
    this->error = this->target-state;
    double u = this->error * this->kp + this->sum_error*this->ki +(this->error - pre_error)*this->kd;
    if(u < this->lower) u = this->lower;
    else if(u > this->upper) u = this->upper;
    this->pre_error = this->error;
    this->sum_error = this->sum_error + this->error;
    return u;
}

/**
 * 重置控制器
*/
void PID_controller::reset(){
    error = 0.0, pre_error = 0.0; sum_error=0.0;
}

/**
 * 设置累计误差
*/
void PID_controller::setSumError(double sum_error){
    this->sum_error = sum_error;
}