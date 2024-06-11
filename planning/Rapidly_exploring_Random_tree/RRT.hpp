/**
 * @date 2024/4/30
*/

#ifndef PLANNING_CONTROL_AUTO_RRT_H
#define PLANNING_CONTROL_AUTO_RRT_H

#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <algorithm>
#include <stdlib.h>
#include <time.h>

#include "../../matplotlibcpp.h"
namespace plt = matplotlibcpp;

using namespace std;
using namespace Eigen;

#define PI 3.14159265354

class RRT{
public:
    struct Node{
        double x, y; //节点坐标
        vector<double>path_x = {}, path_y = {};  //保存找到的路径
        Node(double x, double y);
        Node* parent;
        double cost;

    };
public:

    vector<vector<double>> obstacle_list; //障碍物位置列表[[x, y,size]]
    vector<double> rand_area,play_area; //采样区域x,y 属于【min,max】
    double robot_radius; //机器人半径
    double expand_dis; //扩展步长
    double goal_sample_rate; //采样目标点的概率。百分制，5：代表5%的概率直接采样目标点
    vector<Node*> node_list;
    Node* begin; //根节点
    Node* end; //终结点

    int max_iter;  //确定最大迭代次数

public:

    RRT(const vector<vector<double>> &obstacleList, const vector<double> &randArea,
        const vector<double> &playArea, double robotRadius, double expandDis,
        double goalSampleRate, int MaxIter);

    vector<double>calDistanceAngle(Node * from_node, Node* to_node); //计算两个节点之间的距离和方位角

    bool obstacleFree(Node* node); //判断是否有障碍物

    bool isInsidePlayArea(Node* node); //判断是否在可行区域

    int getNearestNodeIndex(vector<Node*> node_list, Node*rnd_node); //计算最近节点

    Node* sampleFree(); //采样生成节点

    double calDistToGoal(double x, double y);  //计算(x, y)离目标点的距离

    pair<vector<double>, vector<double>> generateFinalCourse(double goal_ind);//生成路径，画图

    Node* steer(Node* from_node, Node* to_node, double extend_length=numeric_limits<double>::max()); //连线方向扩展固定步长查找x_new

    pair<vector<double>, vector<double>>planning();
    
    void setBegin(Node* begin);
    
    void setAnEnd(Node* anEnd);

    void plotCircle(double x, double y, double size, string color = "b"); //画圆

    void draw(Node* node = nullptr);

};

#endif