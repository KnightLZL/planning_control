/**
 * PID 控制算法
*/

#include "PID.hpp"
#include "../utils/KinematicModel.hpp"  //车辆运动学模型
#include "../../matplotlibcpp.h"
#include <algorithm>
namespace plt = matplotlibcpp;

/**
 * PID控制器实现路径跟踪
*/

#define PI  3.1415926

/**
 * 得到距离参考轨迹最近的下标索引
 * @param robot_state 机器人的状态(x,y)
 * @param refer_path 参考路径
 * @return 车辆当前定位距离参考轨迹最近点的下标 
*/
double calTargetIndex(vector<double> robot_sate, vector<vector<double>> refer_path){
    vector<double> dists;
    for(vector <double>xy : refer_path){
        double dist = sqrt(pow(xy[0] - robot_sate[0], 2) + pow(xy[1] - robot_sate[1], 2));
        dists.push_back(dist);
    }
    // 找到最短的路径的下标。得到的是最小元素迭代器在整个容器中的偏移量，也就是最小点的下标
    return min_element(dists.begin(), dists.end()) - dists.begin(); 
}

int main(){
    vector<vector<double>> refer_path(1000, vector<double>(2));
    vector<double> ref_x, ref_y; //保存参考线，用以画图
    // 生成轨迹
    for(int i=0; i<1000;  i++){
        refer_path[i][0] = 0.1 * i;
        refer_path[i][1] = 2 * sin(refer_path[i][0] / 3.0);
        ref_x.push_back(refer_path[i][0]);
        ref_y.push_back(refer_path[i][1]);
    }

    // 运动学模型 x, y, psi, v, l, dt
    KinematicModel ugv(0, -1, 0.5, 2, 2, 0.1);

    // PID控制器
    PID_controller PID(2, 0.01, 25, 0., PI/6, -PI/6);

    // 保存车辆实际的运动轨迹
    vector<double>x_, y_;

    // 车辆状态
    vector<double> robot_state(2);

    // 运行500回合
    // plt::figure_size(600, 200);
    for(int i=0; i<500; i++){
        plt::clf(); //清除画框
        robot_state[0] = ugv.x;
        robot_state[1] = ugv.y;

        // 计算最近距离参考线最近的点的下标
        double min_ind = calTargetIndex(robot_state, refer_path);
        // 计算角度
        double alpha = atan2(refer_path[min_ind][1] - robot_state[1], refer_path[min_ind][0]-robot_state[0]);

        double l_d = sqrt(pow(refer_path[min_ind][0] - robot_state[0], 2) + pow(refer_path[min_ind][1] - robot_state[1],2));
        
        double theta_e = alpha - ugv.psi;
        double e_y = -l_d * sin(theta_e);  //计算横向误差
        double delta_f = PID.calOutput(e_y); //计算控制量 车轮转角

        // 更新状态
        ugv.updateState(0, delta_f);
        x_.push_back(ugv.x);
        y_.push_back(ugv.y);

        // 开始画图
        plt::plot(ref_x, ref_y, "b--");
        plt::plot(x_, y_, "r");
        plt::grid(true);
        // plt::ylim(-2.5, 2.5);
        plt::pause(0.01);

    }

    const char* filename = "../pid_demo.png";
    cout<<"Saving result to " <<filename<<endl;
    plt::save(filename);
    plt::show();
    return 0;

}
