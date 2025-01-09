#include <iostream>
#include <robot.h>
#include <unistd.h>
int main() {
    autopicker::Arm arm = autopicker::Arm();
    arm.init(1, 2, 3, 4, 5);
    arm.rotate_clockwise();
    sleep(0.5);
    arm.isextend();
    for(int i = 0;i<100;i++){
        double roll = 0.2; // 弧度
        double hor = 0.3;	// 横向移动，只能为正值
        double ver = -0.25; // 外伸,只能为负值
        double rot = 1; // 小臂旋转角度,单位为弧度
        double linear_velocity = 480.0; // 线速度 40
        double rotational_speed = 480.0; // 旋转速度 40
        arm.MoveToPosition(roll, hor, ver, rot, linear_velocity, rotational_speed);
        roll = 0.0; // 弧度
        hor = 0.7;	// 横向移动，只能为正值
        ver = -0.05; // 外伸,只能为负值
        rot = 0.5; // 小臂旋转角度
        arm.MoveToPosition(roll, hor, ver, rot, linear_velocity, rotational_speed);

        double p[4] = {0.0, 0.0, 0.0, 0.0}; 
        arm.getPosition(p);
        std::cout << p[0] << " " << p[1] << " " << p[2] << " " << p[3] << std::endl;

        //std::string *error = arm.getErrorCode();
        //std::cout <<  "error:" <<error[0] << std::endl;
    }
    
    while (true)
    {
        //std::cout << "errorCode" << arm.getErrorCode() << std::endl;
    }
    
    return 0;
}