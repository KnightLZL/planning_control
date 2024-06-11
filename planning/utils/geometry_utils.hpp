/***
 * 写的判断库
 * @date 2024/5/17
*/

#ifndef PLANNING_CONTROL_AUTO_UTILS_H
#define PLANNING_CONTROL_AUTO_UTILS_H

#include <vector>
#include <cmath>

using namespace std;

struct Point{
    Point(double x_in, double y_in) : x(x_in), y(y_in) {};
    double x, y;
};

double distanceBetweenPoints(const Point &p1, const Point& p2){
    return sqrt(pow(p2.x - p1.x, 2) + pow(p2.y-p1.y, 2));
}

/***
 * 计算p1 p2到p的最近的点
*/
Point closestPointOnSegement(const Point& p1, const Point& p2, const Point& p){
    double l2 = pow(p2.x-p1.x, 2) + pow(p2.y - p1.y, 2);
    double t = ((p.x - p1.x) *(p2.x - p1.x) + (p.y - p1.y) *(p2.y - p1.y)) / l2;
    t = max(0.0, min(1.0, t));

    double closest_x = p1.x + t * (p2.x - p1.x);
    double closest_y = p1.y + t * (p2.y - p1.y);

    return {closest_x, closest_y};
}

#endif