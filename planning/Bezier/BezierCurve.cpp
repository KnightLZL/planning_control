/**
 * @date 2024.6.10
*/

#include "BezierCurve.hpp"

/**
 * 阶乘
*/
double factorial(int n){
    if(n<=1) return 1;
    return factorial(n-1) * n; //递归方式实现阶乘
}

/**
 * 贝塞尔曲线计算公式
 * @param Ps
 * @param t
 * @return 
*/
Vector2d bezierCommon(vector<Vector2d> Ps, double t){
    if(Ps.size() == 1) 
    return Ps[0];

    Vector2d p_t(0., 0.);
    int n = Ps.size() - 1;  //n阶贝塞尔曲线需要n+1个点
    for(int i=0; i<Ps.size(); i++){
        // 贝塞尔曲线计算公式
        double C_n_i = factorial(n) / (factorial(i) * factorial(n-i));
        p_t += C_n_i * pow((1-t),(n-i)) * pow(t, i) *Ps[i];
    }
    return p_t;
}