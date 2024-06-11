/***
 * @date 2024/4/19
*/

#include "Astar.hpp"

Astar::Node::Node(double x, double y, float cost, double parentIndex) : x(x), y(y), cost(cost), parent_index(parentIndex) {}

Astar::Astar(double resolution, double rebotRadius) : resolution(resolution), robot_radius(rebotRadius) {}

/**
 * 得到障碍物
 * **/

void Astar::calObstacleMap(const vector<double> &ox, const vector<double> & oy){
    // 找到障碍中最大和最小的位置坐标，然后建立坐标
    min_x = round(*min_element(ox.begin(), ox.end()));
    min_y = round(*min_element(oy.begin(), oy.end()));
    max_x = round(*max_element(ox.begin(), ox.end()));
    max_y = round(*max_element(oy.begin(), oy.end()));
    
    cout<<"min_x:"<<min_x<<" min_y:"<<min_y<<"  max_x:"<<max_x<<"   max_y:"<<max_y<<endl;

    x_width = round((max_x-min_x) / resolution);
    y_width = round((max_y-min_y) / resolution);
    cout<<"x_width:"<<x_width<<"    y_width:"<<y_width<<endl;

    obstacle_map = vector<vector<bool>>(x_width, vector<bool>(y_width, false));

    // 给障碍物信息图赋值
    for(size_t i =0; i<x_width; i++){
        double x = calPosition(i, min_x);
        for(size_t j= 0;j<y_width; j++){
            double y = calPosition(j, min_y);
            for(size_t k=0; k<ox.size(); k++){
                double d = sqrt(pow(ox[k]-x, 2) + pow(oy[k]-y, 2));
                if(d <=robot_radius){
                    obstacle_map[i][j] = true;
                    break;
                }
            }

        }
    }

}

/***
 * 计算栅格地图中的位置
 * @param index
 * @param minp
 * @return
*/
double Astar::calPosition(double index , double minp){
    double pos = index * resolution + minp;
    return pos;
}

/****
 * 标记移动代价
 * @return 
*/
vector<vector<double>> Astar::getMotionModel(){
    // x y cost
    motion  = {{1,0,1},
               {0,1,1},
               {-1,0,1},
               {0,-1,1},
               {-1,-1,sqrt(2)},
               {-1,1,sqrt(2)},
               {1,-1,sqrt(2)},
               {1,1,sqrt(2)}};
    return motion;
}

/***
 * 计算起点终点的栅格索引
 * @param position
 * @param minp
 * @return 
*/
double Astar::calXyIndex(double position, double minp){
    return round((position - minp) / resolution);
}

/***
 * 计算栅格索引
 * @param node
 * @return 
*/
double Astar::calIndex(Astar::Node *node) {
    return (node->y-min_y)*x_width + (node->x-min_x);
}

/**
 * 判断节点是否有效，即是否超出边界和碰到障碍物
 * @param node
 * @return 
*/
bool Astar::verifyNode(Astar::Node *node) {
    double px = calPosition(node->x,min_x);
    double py = calPosition(node->y,min_y);
    if(px < min_x) return false;
    if(py < min_y) return false;
    if(px >= max_x) return false;
    if(py >= max_y) return false;
    if(obstacle_map[node->x][node->y]) return false;
    return true;
}

/**
 * 计算路径，便于画画
 * @param goal_node
 * @param closed_set
 * @return 
*/

pair<vector<double>, vector<double>> Astar::calFinalPath(Astar::Node *goal_node, map<double, Node *> closed_set){
    vector<double> rx, ry;
    rx.push_back(calPosition(goal_node->x, min_x));
    ry.push_back(calPosition(goal_node->y, min_y));

    double parent_index = goal_node->parent_index;

    while (parent_index != -1)
    {
        Node* node = closed_set[parent_index];
        rx.push_back(calPosition(node->x, min_x));
        ry.push_back(calPosition(node->y, min_y));

        parent_index = node->parent_index;
    }

    return {rx, ry};
    
}

/***
 * 规划路径
 * @param start 起点
 * @param goal 终点
 * @return 规划后的路径
*/
pair<vector<double>, vector<double>> Astar::planning(vector<double> start, vector<double> goal) {
    double sx = start[0], sy=start[1];
    double gx = goal[0], gy = goal[1];
    Node* start_node  = new Node(calXyIndex(sx, min_x), calXyIndex(sy,min_y), 0.0,-1);
    Node* goal_node = new Node(calXyIndex(gx,min_x), calXyIndex(gy,min_y), 0.0,-1);

    map<double, Node*> open_set, closed_set;

    //将起点加入到open set
    open_set[calIndex(start_node)] = start_node;
    // cout<<calIndex(start_node)<<endl;

    Node* current;
    while(true){
        double c_id = numeric_limits<double>::max();
        double cost = numeric_limits<double>::max();
        // 计算代价最小的节点，与dijkstra代码不同的地方，启发函数的计算方式不同
        for(auto it=open_set.begin(); it!=open_set.end(); it++){
            double now_cost = it->second->cost + calHeuristic(goal_node,it->second);
            if(now_cost < cost){
                cost = now_cost;
                c_id = it->first;
            }
        }

    current = open_set[c_id];
    plotGraph(current);

    if(abs(current->x - goal_node->x) < EPS && abs(current->y - goal_node->y) < EPS){
        // 已经找到目标点
        cout<<"Find goal"<<endl;
        goal_node->parent_index = current->parent_index;
        goal_node->cost = current->cost;
        break;
    }

    // 从open set中去除
    auto iter = open_set.find(c_id);
    open_set.erase(iter);
    // 将找到的点加入到closed_set中
    closed_set[c_id] = current;

    //基于运动模式扩展搜索地图
    for(vector<double>move:motion) {
        // cout<<move[0]<<move[1]<<move[2]<<endl;
        Node* node = new Node(current->x + move[0],current->y+ move[1],current->cost+move[2], c_id);
        double n_id = calIndex(node);

        if(closed_set.find(n_id) != closed_set.end()) continue;
        // 已经在了，跳过

        if(!verifyNode(node)) continue; //超出边界或者碰到障碍物

        if(open_set.find(n_id) == open_set.end()){
            // open中没有这个节点
            open_set[n_id] = node;
        }else{
            // 如果open中已经存在了
            if(open_set[n_id]->cost >= node->cost){
                // 更新最小的代价的节点
                open_set[n_id] = node;
                }
            }
        }
    }
    return calFinalPath(goal_node, closed_set);
}

/**
 * 画图
 * @param current
*/
void Astar::plotGraph(Astar::Node *current){
    // plt::clf();
    plt::plot(ox, oy, ".k");
    plt::plot(vector<double>{st[0]}, vector<double>{st[1]}, "og");
    plt::plot(vector<double>{go[0]}, vector<double>{go[1]}, "xb");
    plt::grid(true);
    plt::plot(vector<double>{calPosition(current->x, min_x)}, vector<double>{calPosition(current->y, min_y)}, "xc");
    plt::pause(0.001);

}


//设置起始点信息
void Astar::setSt(const vector<double> &st){
    Astar::st = st;
}

void Astar::setGo(const vector<double> &go){
    Astar::go = go;

}

void Astar::setOx(const vector<double> &ox){
    Astar::ox = ox;
}

void Astar::setOy(const vector<double> & oy){
    Astar::oy = oy;
}

/**
 * 进行启发函数计算
 * @param n1 节点1
 * @param n2 节点2
*/
double Astar::calHeuristic(Astar::Node *n1, Astar::Node *n2){
    double w = 1.0; //启发函数权重
    double d = w*sqrt(pow(n1->x - n2->x, 2) + pow(n1->y - n2->y, 2)); //欧式距离，两点间的直线距离
    // double d = w * (abs(n1->x - n2->x) + abs(n1->y - n2->y)); //曼哈顿距离
    return d;
}