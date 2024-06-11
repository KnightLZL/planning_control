/**
 * @date 2024/4/30
 * 测试Astar算法
*/
#include "Astar.hpp"


int main(){
    vector<double>start{-5,5}, goal{50,50}; //设置起始点信息
    double grid_size = 5.0;  //地图栅格大小
    double rebot_radius = 2.0; //检测碰撞的圆半径

    vector<double> ox;
    vector<double> oy;

    // 添加障碍物
    for(double i = -10; i<60; i++){
        ox.push_back(i);
        oy.push_back(-10.0);
    }
    for(double i=-10; i<60;i++){
        ox.push_back(60.0);
        oy.push_back(i);
    }
    for(double i=-10; i<61; i++){
        ox.push_back(i);
        oy.push_back(60.0);
    }
    for(double i=-10; i<61; i++){
        ox.push_back(-10.0);
        oy.push_back(i);
    }
    for(double i=-10; i<40;i++){
        ox.push_back(20.0);
        oy.push_back(i);
    }
    for(double i=0;i<40;i++){
        ox.push_back(40.0);
        oy.push_back(60.0-i);
    }

    ox.push_back(30);
    oy.push_back(30);

    // 创建astar类
    Astar astar(grid_size, rebot_radius);
    astar.setGo(goal); //设置目标点
    astar.setSt(start); //设置起点
    astar.setOx(ox);  //设置障碍物
    astar.setOy(oy);

    astar.calObstacleMap(ox,oy); //计算障碍物地图
    astar.getMotionModel();   //获得运动模式

    // 获取规划后的路径
    pair<vector<double>, vector<double>> xy = astar.planning(start, goal);
    // 画图
    plt::plot(xy.first, xy.second, "-r");
    
    const char* filename = "./astar_demo_oushi.png";
    cout<<"Saving result to"<<filename<<std::endl;
    plt::save(filename);
    plt::show();

    return 0;

}