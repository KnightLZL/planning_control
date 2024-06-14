
#include "BezierCurve.h"
#include "../../matplotlibcpp.h"
namespace plt = matplotlibcpp;

int main(){
    // 需要平滑的点
    // 五个原始点，其实为四阶贝塞尔曲线
        // vector<Vector2d>Ps{Vector2d (9.036145, 51.779661),Vector2d(21.084337, 70.084746),Vector2d(37.607573, 50.254237),Vector2d(51.893287, 69.745763),Vector2d(61.187608,  49.576271)};
    vector<Vector2d>Ps{Vector2d (0,0),Vector2d (3,1),Vector2d (5, 0), Vector2d (5,2), Vector2d (10,2)}; 

    // 将原始的保存为参考线
    vector<double> x_ref, y_ref;
    for(int i=0; i<Ps.size(); i++){
        x_ref.push_back(Ps[i][0]);
        y_ref.push_back(Ps[i][1]);
    }

    vector<double>x_, y_;
    // 设置视图大小
    plt::figure_size(1600, 800);
    for(int t=0; t<100;t++){
        // 增密后为100个点
        // 清除当前figure
        plt::clf();
        Vector2d pos = bezierCommon(Ps, (double)t/100);
        x_.push_back(pos[0]);
        y_.push_back(pos[1]);


        // 画图
        plt::plot(x_, y_, "r");
        plt::plot(x_ref, y_ref, "k"); //k 为黑色
        // 暂停0.01s后再绘制
        plt::pause(0.01);
    }

    // 保存图像
    const char* filename = "../bezier_demo.png";
    cout<<"Saving result to "<<filename<<std::endl;
    plt::save(filename);
    plt::show();
    return 0;
}
