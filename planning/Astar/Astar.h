/***
 * @author ZLLee
 * @brief Astar 路径规划算法
 * @date 2024/04/01 
 ***/
// 防止头文件重复要引用

#ifndef PLANNING_CONTROL_AUTO_ASTAR_H
#define PLANNING_CONTROL_AUTO_ASTAR_H

// 开始包含所必须的头文件
#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <algorithm>
#include <stdlib.h>
#include <time.h>
#include <map>
#include "../../matplotlibcpp.h"

namespace plt = matplotlibcpp;
using namespace std;
using namespace Eigen;

#define EPS  1e-4
#define PI 3.1415926

// 定义a星算法的类
class Astar{
public:
    struct Node{
        double x;
        double y;
        float cost;  //代价
        double parent_index;  //前一个父节点

        //自定义构造函数
        Node(double x, double y, float cost, double parentIndex);
    };

private:
    double resolution;  //栅格大小
    double robot_radius;
    double min_x, min_y, max_x, max_y; //地图范围
    double x_width, y_width;  //长宽
    vector<vector<bool>> obstacle_map; //障碍物地图
    vector<vector<double>>motion; //障碍物地图
    vector<double> st,go;
    vector<double>ox, oy;

public:
    Astar(double resolution, double rebotRadius);

    void calObstacleMap(const vector<double> &ox, const vector<double> & oy);

    double calPosition(double index, double minp);

    vector<vector<double>>getMotionModel();
    
    double calXyIndex(double calPosition, double minp);

    double calIndex(Node*node);
    bool verifyNode(Node*node);

    pair<vector<double>,vector<double>> calFinalPath(Node*goal_node, map<double, Node*>closed_set);

    pair<vector<double>, vector<double>> planning(vector<double>start, vector<double>goal);

    double calHeuristic(Node* n1, Node* n2);

    void plotGraph(Node*current);

    void setSt(const vector<double> &st);
    void setGo(const vector<double> &go);
    void setOx(const vector<double> &ox);
    void setOy(const vector<double> &oy);    
};

#endif