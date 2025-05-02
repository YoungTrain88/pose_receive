// #include <iostream>
// #include <robot.h>
// #include <unistd.h>
// int main() {
//     autopicker::Arm arm = autopicker::Arm();
//     arm.init(1, 2, 3, 4, 5);
//     //arm.rotate_clockwise();
//     //sleep(0.5);
//     //arm.isextend();



//     for(int i = 0;i<10;i++){
//         double roll = 0.0; // 弧度
//         double hor = 0.5;	// 横向移动，只能为正值
//         double ver = -0; // 外伸,只能为负值
//         double rot = 0; // 小臂旋转角度,单位为弧度
//         double linear_velocity = 40.0; // 线速度 40
//         double rotational_speed = 40.0; // 旋转速度 40

//         arm.MoveToPosition(roll, hor, ver, rot, linear_velocity, rotational_speed);
//         // roll = 0.0; // 弧度
//         // hor = 0.7;	// 横向移动，只能为正值
//         // ver = -0.05; // 外伸,只能为负值
//         // rot = 0.5; // 小臂旋转角度
//         // arm.MoveToPosition(roll, hor, ver, rot, linear_velocity, rotational_speed);

//         double p[4] = {0.0, 0.0, 0.0, 0.0}; 
//         arm.getPosition(p);
//         std::cout << p[0] << " " << p[1] << " " << p[2] << " " << p[3] << std::endl;

//         //std::string *error = arm.getErrorCode();
//         //std::cout <<  "error:" <<error[0] << std::endl;
//     }
//     arm.stop();
    
    
//     //while (true)
//     //{
//         // double roll = 0.2; // 弧度
//         // double hor = 0.3;	// 横向移动，只能为正值
//         // double ver = -0.25; // 外伸,只能为负值
//         // double rot = 1; // 小臂旋转角度,单位为弧度
//         // double linear_velocity = 40.0; // 线速度 40
//         // double rotational_speed = 40.0; // 旋转速度 40
//         // arm.MoveToPosition(roll, hor, ver, rot, linear_velocity, rotational_speed);
//         // roll = 0.0; // 弧度
//         // hor = 0.7;	// 横向移动，只能为正值
//         // ver = -0.05; // 外伸,只能为负值
//         // rot = 0.5; // 小臂旋转角度
//         // arm.MoveToPosition(roll, hor, ver, rot, linear_velocity, rotational_speed);

//         // double p[4] = {0.0, 0.0, 0.0, 0.0}; 
//         // arm.getPosition(p);
//         // std::cout << p[0] << " " << p[1] << " " << p[2] << " " << p[3] << std::endl;
//         // //std::cout << "errorCode" << arm.getErrorCode() << std::endl;
//     //}
    
//     return 0;
// }
#include <iostream>
#include <robot.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <cstring>
#include <nlohmann/json.hpp> // JSON 库

using json = nlohmann::json;

int main() {
    // 初始化机械臂
    autopicker::Arm arm = autopicker::Arm();
    arm.init(1, 2, 3, 4, 5);

    // 创建套接字
    int server_fd, new_socket;
    struct sockaddr_in address;
    int addrlen = sizeof(address);
    char buffer[1024] = {0};

    server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd == 0) {
        std::cerr << "Socket creation failed!" << std::endl;
        return -1;
    }

    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(12345);

    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0) {
        std::cerr << "Bind failed!" << std::endl;
        return -1;
    }

    if (listen(server_fd, 3) < 0) {
        std::cerr << "Listen failed!" << std::endl;
        return -1;
    }

    std::cout << "Waiting for connection..." << std::endl;
    new_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t *)&addrlen);
    if (new_socket < 0) {
        std::cerr << "Accept failed!" << std::endl;
        return -1;
    }
    std::cout << "Connection established!" << std::endl;

    while (true) {
        // 接收数据
        int valread = read(new_socket, buffer, 1024);
        if (valread > 0) {
            // 解析 JSON 数据
            std::string json_data(buffer, valread);
            json received_data = json::parse(json_data);

            // 提取数据
            double roll = received_data["shoulder_arm_angles"];
            double hor = 0.5;
            double ver = 0;
            double rot = received_data["arm_angles"];
            double linear_velocity = 40;
            double rotational_speed = 40;

            // 执行机械臂动作
            arm.MoveToPosition(roll, hor, ver, rot, linear_velocity, rotational_speed);

            // 获取机械臂位置
            double p[4] = {0.0, 0.0, 0.0, 0.0};
            arm.getPosition(p);
            std::cout << "Position: " << p[0] << " " << p[1] << " " << p[2] << " " << p[3] << std::endl;
        }

        // 清空缓冲区
        memset(buffer, 0, sizeof(buffer));
    }

    arm.stop();
    close(new_socket);
    close(server_fd);
    return 0;
}