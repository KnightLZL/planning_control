/**
 * @date 2024.6.14
 * @author ZLLee
 * @brief PID控制算法
*/

#ifndef PLANNING_CONTROL_AUTO_PID_H
#define PLANNING_CONTROL_AUTO_PID_H

using namespace std;

/**
 * 位置式PID实现
*/
class PID_controller{
private:
    double kp, ki, kd, target, upper, lower;
    double error = 0.0, pre_error=0.0, sum_error=0.0;
public:
    // 构造函数
    PID_controller(double kp, double ki, double kd, double target, double upper, double lower);

    // 参数设置接口
    void setTarget(double target);

    // 设置调整参数
    void setK(double kp, double ki, double kd);

    // 设置输出量的上下限
    void setBound(double upper, double lower);

    // 计算输出
    double calOutput(double state);

    // 重置
    void reset();

    // 计算误差
    void setSumError(double sum_error);

};

#endif