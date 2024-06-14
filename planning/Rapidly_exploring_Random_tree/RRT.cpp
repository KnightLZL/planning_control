/**
 * @date 2024/5/1
*/

#include "RRT.h"
#include "../utils/geometry_utils.hpp"

RRT::Node::Node(double x, double y) : x(x), y(y), parent(nullptr), cost(0) {};

RRT::RRT(const vector<vector<double>> &obstacleList, const vector<double> &randArea, const vector<double> &playArea, double robotRadius, double expandDis, double goalSampleRate, int MaxIter) : obstacle_list(obstacleList), rand_area(randArea),play_area(playArea), robot_radius(robotRadius),expand_dis(expandDis), goal_sample_rate(goalSampleRate), max_iter(MaxIter) {};

/**
 * 计算两个节点之间的距离和方位角
 * @param from_node
 * @param to_node
 * @return 
*/
vector<double> RRT::calDistanceAngle(RRT::Node* from_node, RRT::Node* to_node){
    double dx = to_node->x - from_node->x;
    double dy = to_node->y - from_node->y;
    double d = sqrt(pow(dx, 2) + pow(dy,2));
    double theta = atan2(dy, dx);
    return {d, theta};
}


/**
 * 判断是否有障碍物（using）,使用线段与椭圆的相交性方法来实现
 * @param node
 * @return 
*/
bool RRT::obstacleFree(Node* node){
    auto isSegmentIntersectingObstacle = [this](const Point& p1, const Point&p2){
        for (const auto& obs : obstacle_list){
            Point obstacleCenter = {obs[0], obs[1]};
            double distance = distanceBetweenPoints(p1, p2);
            const Point closestPointOnLine = closestPointOnSegement(p1, p2, obstacleCenter);
            double dist_to_center = distanceBetweenPoints(obstacleCenter, closestPointOnLine);
            if(dist_to_center <= obs[2] + robot_radius && dist_to_center >=0 && dist_to_center <=distance){
                return true;
            }
        }
        return false;
    };

    for(int i=0; i<node->path_x.size()-1; i++){
        Point point1 = {node->path_x[i], node->path_y[i]};
        Point point2 = {node->path_x[i+1], node->path_y[i+1]};

        if(isSegmentIntersectingObstacle(point1, point2)){
            //障碍物
            return false;
        }
    }
    // 安全的
    return true;

}

/***
 * 判断是否在可行区域内
 * @param node
 * @return
*/
bool RRT::isInsidePlayArea(RRT::Node* node){
    if(node->x <play_area[0] || node ->x > play_area[1] || node->y < play_area [2] || node->y > play_area[3]){
        return false;
    }
    return true;
}

/***
 * 计算最近的节点
 * @param node_list 节点列表
 * @param rnd_node 随机采样的节点
 * @return 最近的节点索引
*/
int RRT::getNearestNodeIndex(vector<Node*> node_list, RRT::Node* rnd_node){
    int min_index = -1;
    double d  = numeric_limits<double>::max();
    for (int i = 0; i<node_list.size(); i++){
        Node* node = node_list[i];
        double dist = pow(node->x - rnd_node->x, 2) + pow(node->y - rnd_node->y, 2);
        if(d > dist){
            d = dist;
            min_index = i;
        }
    }
    return min_index;
}

/***
 *  以（100 - goal_sample_rate）%的概率进行随机生长， （goal_sample_rate）%的概率向目标点生长
*/
RRT::Node* RRT::sampleFree(){
    Node* rnd = nullptr;
    if(rand() % (100) > goal_sample_rate){
        double min_rand = rand() / double(RAND_MAX) * (rand_area[1] - rand_area[0]) + rand_area[0];
        double max_rand = rand() / double(RAND_MAX) * (rand_area[1] - rand_area[0]) + rand_area[0];
        rnd = new Node(min_rand, max_rand);
    }else {
        rnd = new Node(end->x, end->y);
    }
    return rnd;
}

void RRT::setBegin(RRT::Node* begin){
    RRT::begin = begin;
}

void RRT::setAnEnd(RRT::Node* anEnd){
    RRT::end = anEnd;
}

/**
 * 计算(x,y)离目标点的距离
 * @param x
 * @param y
 * @return
*/
double RRT::calDistToGoal(double x, double y){
    double dx = x - end->x;
    double dy = y - end->y;
    return sqrt(pow(dx, 2) + pow(dy, 2));
}

/**
 * 生成路径
 * @param goal_ind
 * @return
*/
pair<vector<double>, vector<double>> RRT::generateFinalCourse(double goal_ind){
    vector<double> x_,y_;
    x_.push_back(end->x);
    y_.push_back(end->y);
    Node* node = node_list[goal_ind];
    while(node->parent != nullptr){
        x_.push_back(node->x);
        y_.push_back(node->y);
        node = node->parent;
    }
    x_.push_back(node->x);
    y_.push_back(node->y);
    return {x_, y_};
}

/**
 * 连线方向扩展固定步长查找x_new
 * @param from_node x_near
 * @param to_node  x_rand
 * @param extend_length 扩展步长，默认是Inf
*/
RRT::Node* RRT::steer(RRT::Node* from_node, RRT::Node* to_node, double extend_length){
    // 利用反正切计算角度，然后利用角度和步长计算新坐标
    vector<double> dist_angle = calDistanceAngle(from_node, to_node);

    double new_x, new_y;
    if(extend_length >= dist_angle[0]){
        new_x = to_node->x;
        new_y = to_node->y;
    }else{
        new_x = from_node->x + cos(dist_angle[1]) * extend_length;
        new_y = from_node->y + sin(dist_angle[1]) * extend_length;
    }

    Node* new_node = new Node(new_x, new_y);
    new_node->path_x.push_back(from_node->x);
    new_node->path_y.push_back(from_node->y);
    new_node->path_x.push_back(to_node->x);
    new_node->path_y.push_back(to_node->y);

    new_node->parent = from_node;

    return new_node;
}

/**
 * RRT 路径规划
 * @return 轨迹数据
*/
pair<vector<double>, vector<double>> RRT::planning(){
    node_list.push_back(begin); //将起点作为根节点x_{init}，加入到随机树的节点集合中
    for(int i=0; i<max_iter; i++){
        // 从可行区域内随机选取一个节点x_{rand}
        Node* rnd_node = sampleFree();

        // 从已经生成的树中利用欧式距离判断距离x_{rand}最近的点x_{near}
        int nearest_ind = getNearestNodeIndex(node_list, rnd_node);
        Node* nearest_node = node_list[nearest_ind];

        // 从x_{near}与x_{rand}的连线方向上扩展固定步长u，得到新节点 x_{new}
        Node* new_node = steer(nearest_node, rnd_node, expand_dis);

        // 如果在可行区域内，切x_{near}与x_{new}之间没有障碍物
        if(isInsidePlayArea(new_node) && obstacleFree(new_node)){
            node_list.push_back(new_node);
        }

        // 如果此时得到的节点x_new到目标点的距离小于扩展步长，则直接将目标点作为x_rand
        if(calDistToGoal(node_list[node_list.size() - 1]->x, node_list[node_list.size()-1]->y)  <= expand_dis){
            Node* final_node = steer(node_list[node_list.size()-1], end, expand_dis);
            if(obstacleFree(final_node)) {
                cout<<"达到目标点"<<endl;
                return generateFinalCourse(node_list.size()-1);
            }
        }
        draw(rnd_node);
    }
    return {};
}

/**
 * 画图
 * @param x
 * @param y
 * @param size
 * @param color
*/
void RRT::plotCircle(double x, double y, double size, string color){
    vector<double> x_t, y_t;
    for(double i =0; i<=2*PI; i+=0.01){
        x_t.push_back(x+size*cos(i));
        y_t.push_back(y+size*sin(i));
    }
    plt::plot(x_t, y_t, color);
}

/**
 * 画出搜索过程的图
 * @param node
*/
void RRT::draw(Node* node){
    plt::clf();
    // 画出随机点
    if(node){
        plt::plot(vector<double>{node->x}, vector<double>{node->y}, "^k");
        if(robot_radius > 0){
            plotCircle(node->x, node->y, robot_radius, "-r");
        }
    }

    // 画出已经生成的树
    for(Node* node1: node_list){
        if(node1->parent){
            plt::plot(node1->path_x, node1->path_y, "-g");
        }
    }

    // 画障碍物
    for(vector<double> ob:obstacle_list){
        plotCircle(ob[0], ob[1], ob[2]);
    }

    plt::plot(vector<double>{play_area[0], play_area[1], play_area[1],play_area[0], play_area[0]},
                vector<double>{play_area[2], play_area[2], play_area[3],
                play_area[3], play_area[2]}, "k-");

    // 画出起点和目标点
    plt::plot(vector<double>{begin->x}, vector<double>{begin->y},"xr");
    plt::plot(vector<double>{end->x}, vector<double>{end->y}, "xr");
    plt::axis("equal");
    plt::grid(true);
    plt::xlim(play_area[0]-1, play_area[1]+1);
    plt::ylim(play_area[2]-1, play_area[3]+1);
    plt::pause(0.01);
}


