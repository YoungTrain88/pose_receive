#include <iostream>
#include <robot.h>
#include <unistd.h>
int main() {
    autopicker::Arm arm = autopicker::Arm();
    arm.init(1, 2, 3, 4, 5);
    arm.extend();
    sleep(0.05);
    arm.retract();
    // for(int i = 0;i<100;i++){
    //     double roll = 0.2; // 弧度
    //     double hor = 0.3;	// 横向移动，只能为正值
    //     double ver = -0.2; // 外伸,只能为负值
    //     double rot = 0.2; // 小臂旋转角度
    //     double linear_velocity = 160.0; // 线速度 40
    //     double rotational_speed = 160.0; // 旋转速度 40
    //     arm.MoveToPosition(roll, hor, ver, rot, linear_velocity, rotational_speed);
    //     roll = 0.0; // 弧度
    //     hor = 0.7;	// 横向移动，只能为正值
    //     ver = -0.1; // 外伸,只能为负值
    //     rot = 0.0; // 小臂旋转角度
    //     arm.MoveToPosition(roll, hor, ver, rot, linear_velocity, rotational_speed);

    //     double p[4] = {0.0, 0.0, 0.0, 0.0}; 
    //     arm.getPosition(p);
    //     std::cout << p[0] << " " << p[1] << " " << p[2] << std::endl;

    //     //std::string *error = arm.getErrorCode();
    //     //std::cout <<  "error:" <<error[0] << std::endl;
    // }
    // //double *p = arm.getPosition();
    //     // std::cout << p[0] << " " << p[1] << " " << p[2] << std::endl;
    // while (true)
    // {
    //     //std::cout << "errorCode" << arm.getErrorCode() << std::endl;
    // }
    
    return 0;
}