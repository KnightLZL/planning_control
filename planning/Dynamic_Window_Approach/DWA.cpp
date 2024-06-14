/**
 * @date 2024.6.10
*/

#include "DWA.h"

// DWA构造函数 + 赋值
DWA::DWA(double dt, double vMin, double vMax, double wMin, double wMax, double predictTime, double aVmax, double aWmax, double vSample, double wSample, double alpha, double beta, double gamma, double radius, double judgeDistance): dt(dt), v_min(vMin), v_max(vMax), w_min(wMin), w_max(wMax), predict_time(predictTime), a_vmax(aVmax),
          a_wmax(aWmax), v_sample(vSample), w_sample(wSample), alpha(alpha), beta(beta), gamma(gamma), radius(radius),
          judge_distance(judgeDistance) {}

/**
 * 计算速度边界限制Vm
 * @return 速度边界限制后的速度空间Vm
 */
vector<double> DWA::calVelLimit() {
    return {v_min,v_max,w_min,w_max };
}


/**
 * 计算加速度限制vd
*/
vector<double> DWA::calAccelLimit(double v, double w){
    double v_low = v - a_vmax*dt;
    double v_high = v + a_vmax*dt;
    double w_low = w - a_wmax*dt;
    double w_high = w + a_wmax*dt;
    return {v_low, v_high, w_low, w_high};
}

/**
 * 环境障碍物限制va
 * @param state 当前机器人状态
 * @param obstacle 障碍物位置
 * @return 移动机器人不与周围障碍物发生碰撞的速度空间Va
*/
vector<double> DWA::calObstacleLimit(VectorXd state, vector<Vector2d>obstacle){
    double v_low = v_min;
    double v_high = sqrt(2*_dist(state, obstacle)*a_vmax);
    double w_low = w_min;
    double w_high = sqrt(2*_dist(state, obstacle)*a_wmax);
    return {v_low, v_high, w_low, w_high};
}

/**
 * 速度采样,得到速度空间窗口
 * @param v 当前时刻线速度
 * @param w 当前时刻角速度
 * @param state 当前机器人状态
 * @param obstacle 障碍物位置
 * @return [v_low,v_high,w_low,w_high]: 最终采样后的速度空间
 */
vector<double> DWA::calDynamicWindowVel(double v, double w,VectorXd state, vector<Vector2d> obstacle) {


    vector<double> Vm = calVelLimit();
    vector<double> Vd = calAccelLimit(v,w);
    vector<double> Va = calObstacleLimit(state,obstacle);
    double a = max({Vm[0],Vd[0],Va[0]});
    double b = min({Vm[1],Vd[1],Va[1]});
    double c = max({Vm[2], Vd[2],Va[2]});
    double d = min({Vm[3], Vd[3],Va[3]});
    //得到线速度和角速度的限制区域
    return {a,b,c,d};
}

/**
 * 计算当前移动机器人距离障碍物最近的几何距离
 * @param state 当前机器人状态
 * @param obstacle 所有障碍物位置
 * @return 移动机器人距离障碍物最近的几何距离
 */
double DWA::_dist(VectorXd state, vector<Vector2d> obstacle) {
    double min_dist = 100000;
    for(Vector2d obs:obstacle){
        //state.head(2)表示机器人状态向量中的前两个元素，即机器人的x和y坐标。然后，(obs-state.head(2))计算了障碍物位置与机器人位置之间的向量差。最后，.norm()方法计算了这个向量的二范数，即欧氏距离。这行代码的作用是计算机器人当前位置与障碍物位置之间的距离，用于确定最小距离。这样的距-离计算方法在路径规划和避障算法中经常被使用。
        double distance = (obs-state.head(2)).norm();
        min_dist = distance>min_dist?min_dist:distance;
    }
    return min_dist;
}

/**
 * 机器人运动学模型
 * @param state 状态量---x,y,yaw,v,w
 * @param control 控制量---v,w,线速度和角速度
 * @param dt 采样时间
 * @return 下一步的状态
 */
VectorXd DWA::kinematicModel(VectorXd state, vector<double> control, double dt) {
    state(0) += control[0] * cos(state(2)) * dt;
    state(1) += control[0] * sin(state(2)) * dt;
    state(2) += control[1] * dt;
    state(3) = control[0];
    state(4) = control[1];

    return state;
    }

/**
 * 轨迹推算
 * @param state 当前状态---x,y,yaw,v,w
 * @param v 当前时刻线速度
 * @param w 当前时刻线速度
 * @return 推算后的轨迹
 */
vector<VectorXd> DWA::trajectoryPredict(VectorXd state, double v, double w) {
    vector<VectorXd> trajectory;
    trajectory.push_back(state);
    double time =0;
    while(time<=predict_time){
        state = kinematicModel(state,{v,w},dt);
        trajectory.push_back(state);
        time+=dt;
    }
    return trajectory;
}


/**
 * 轨迹评价函数,评价越高，轨迹越优
 * @param state 当前状态---x,y,yaw,v,w
 * @param goal 目标点位置，[x,y]
 * @param obstacle 障碍物位置，dim:[num_ob,2]
 * @return 最优控制量、最优轨迹
 */
pair<vector<double>, vector<VectorXd>>DWA::trajectoryEvaluation(VectorXd state, Vector2d goal, vector<Vector2d> obstacle) {
    double G_max = -10000000; //最优评价
    vector<VectorXd> trajectory_opt; //最优轨迹
    trajectory_opt.push_back(state);
    vector<double>control_opt={0.,0.}; // 最优控制
    vector<double>dynamic_window_vel = calDynamicWindowVel(state(3),state(4),state,obstacle);//第1步--计算速度空间

    // double sum_heading =0.0,sum_dist=0.0,sum_vel=0.0;//统计全部采样轨迹的各个评价之和，便于评价的归一化,
    // double v = dynamic_window_vel[0];
    // double w = dynamic_window_vel[2];
    // while(v<dynamic_window_vel[1]){
    //     while(w<dynamic_window_vel[3]){
    //         //求取每个路径上点的每个评价指标的总和
    //         vector<VectorXd>trajectory = trajectoryPredict(state,v,w);
    //         double heading_eval = alpha*_heading(trajectory,goal);
    //         double dist_eval = beta*_distance(trajectory,obstacle);
    //         double vel_eval = gamma*_velocity(trajectory);
    //         sum_vel+=vel_eval;
    //         sum_dist+=dist_eval;
    //         sum_heading+=heading_eval;
    //         w+=w_sample;
    //     }
    //     v+=v_sample;
    // }
    double sum_heading =1.0,sum_dist=1.0,sum_vel=1.0;//不进行归一化
    double v = dynamic_window_vel[0]; //线速度最小值
    double w = dynamic_window_vel[2]; //角速度最小值
    // 只要小于线速度和角速度的最大值就继续循环
    while(v<dynamic_window_vel[1]){
        while(w<dynamic_window_vel[3]){

            // 开始计算每条路径的评价指标
            vector<VectorXd>trajectory = trajectoryPredict(state,v,w);//第2步--轨迹推算

            double heading_eval = alpha*_heading(trajectory,goal)/sum_heading;
            double dist_eval = beta*_distance(trajectory,obstacle)/sum_dist;
            double vel_eval = gamma*_velocity(trajectory)/sum_vel;
            double G = heading_eval+dist_eval+vel_eval; // 第3步--轨迹评价

            if(G_max<=G){
                G_max = G;
                trajectory_opt=trajectory; //保存最优轨迹和最优控制量
                control_opt={v,w};
            }
            w+=w_sample;
        }
        v+=v_sample;
        w = dynamic_window_vel[2]; //将加速度重新置为最小值，开始搜索
    }
    return {control_opt,trajectory_opt};

}

/**
 * 方位角评价函数
 * 评估在当前采样速度下产生的轨迹终点位置方向与目标点连线的夹角的误差
 * @param trajectory 轨迹，dim:[n,5]
 * @param goal 目标点位置[x,y]
 * @return 方位角评价数值
 */
double DWA::_heading(vector<VectorXd> trajectory, Vector2d goal) {
    Vector2d dxy = goal-trajectory[trajectory.size()-1].head(2);
    double error_angle = atan2(dxy(1),dxy(0)); //计算角度
    // 计算角度与规律端点的误差
    double cost_angle = error_angle-trajectory[trajectory.size()-1](2);//（2）位轨迹端点的横摆角
    // 评价越高，轨迹越好，因此将误差反过来
    double cost = PI-abs(cost_angle);
    return cost;
}

/**
 * 速度评价函数
 * 表示当前的速度大小，可以用模拟轨迹末端位置的线速度的大小来表示
 * @param trajectory 轨迹，dim:[n,5]
 * @return 速度评价值
 */
double DWA::_velocity(vector<VectorXd> trajectory) {
    return trajectory[trajectory.size()-1](3);  //速度v
}

/**
 * 距离评价函数
 * 表示当前速度下对应模拟轨迹与障碍物之间的最近距离；
 * 如果没有障碍物或者最近距离大于设定的阈值，那么就将其值设为一个较大的常数值。
 * @param trajectory 轨迹，dim:[n,5]
 * @param obstacle 障碍物位置，dim:[num_ob,2]
 * @return 距离评价值
 */
double DWA::_distance(vector<VectorXd> trajectory, vector<Vector2d> obstacle) {
    double min_r = 10000000;
    for(Vector2d obs:obstacle){
        for(VectorXd state:trajectory){
            Vector2d dxy = obs-state.head(2); //head(2)前两个是位置(x,y)
            double r = dxy.norm(); //2范数就是求欧式距离
            min_r = min_r>r?r:min_r; //找到最短距离
        }
    }
    if(min_r<radius+0.2){
        return min_r;
    }else{
        return judge_distance;
    }
}

/**
 * 滚动窗口算法控制器
 * @param state 机器人当前状态--[x,y,yaw,v,w]
 * @param goal 目标点位置，[x,y]
 * @param obstacle 障碍物位置，dim:[num_ob,2]
 * @return  最优控制量[v,w]、最优轨迹
 */
pair<vector<double>, vector<VectorXd>> DWA::dwaControl(VectorXd state, Vector2d goal , vector<Vector2d> obstacle){
    pair<vector<double>, vector<VectorXd>> res = trajectoryEvaluation(state, goal, obstacle);
    return res;
}