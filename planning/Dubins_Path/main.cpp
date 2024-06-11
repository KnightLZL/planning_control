/***
 * @date 2023.6.9
*/
#include "Dubins.hpp"

int main(){
    Vector3d start(1.0,1.0,(double)45/180*PI);
    Vector3d goal(-3.0,-3.0,(double)-45/180*PI);
    double curvature = 1;  //曲率为1时，路径段的曲率与路径段长度相等，这意味着路径段是以单位曲率圆为基础进行规划的
    double step_size = 0.1;  //平滑是的间隔
    Dubins dubins;
    Dubins::ResultDubins rd = dubins.dubins_path_planning(start, goal, curvature, step_size);
    plt::plot(rd.p_x,rd.p_y);
    plt::plot(vector<double>{start[0]},vector<double>{start[1]},"og");
    plt::plot(vector<double>{goal[0]},vector<double>{goal[1]},"xb");
    plt::title("mode: "+rd.mode);

    const char* filename = "./dubins_demo.png";
    cout << "Saving result to " << filename << std::endl;
    plt::save(filename);
    plt::show();
    cout<<"mode: "<<rd.mode<<endl;
    return 0;
}