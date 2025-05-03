#include <iostream>
#include <string>
#include <sstream>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <nlohmann/json.hpp> // JSON 库
#include "robot.h"

using json = nlohmann::json;

int main() {
    autopicker::Arm arm = autopicker::Arm();
    arm.init(1, 2, 3, 4, 5);

    int server_fd, new_socket;
    struct sockaddr_in address;
    int addrlen = sizeof(address);
    char buffer[2048] = {0};

    // 创建套接字
    server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd == 0) {
        std::cerr << "Socket creation failed!" << std::endl;
        return -1;
    }

    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(12345);

    // 绑定套接字
    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0) {
        std::cerr << "Socket bind failed!" << std::endl;
        return -1;
    }

    // 开始监听
    if (listen(server_fd, 3) < 0) {
        std::cerr << "Socket listen failed!" << std::endl;
        return -1;
    }

    std::cout << "Waiting for connection..." << std::endl;

    new_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t *)&addrlen);
    if (new_socket < 0) {
        std::cerr << "Socket accept failed!" << std::endl;
        return -1;
    }

    std::cout << "Connection established!" << std::endl;

    while (true) {
        int valread = read(new_socket, buffer, 1024);
        if (valread > 0) {
            buffer[valread] = '\0'; // 确保字符串以 '\0' 结尾
            std::string data(buffer);

            // 按行解析数据
            std::istringstream ss(data);
            std::string line;
            double right_arm_angle = 0.0, right_shoulder_arm_angle = 0.0;

            while (std::getline(ss, line)) {
                
                try {
                    // 解析数据，假设格式为 "right_arm:0.0,right_shoulder:0.0"
                    std::istringstream line_stream(line);
                    std::string key_value_pair;

                    while (std::getline(line_stream, key_value_pair, ',')) {
                        std::istringstream pair_stream(key_value_pair);
                        std::string key;
                        double value;

                        if (std::getline(pair_stream, key, ':')) {
                            // 去掉键名中的多余双引号
                            if (!key.empty() && key.front() == '"') {
                                key.erase(0, 1); // 删除开头的双引号
                            }
                            if (!key.empty() && key.back() == '"') {
                                key.pop_back(); // 删除结尾的双引号
                            }

                            // std::cout << "Key: " << key << std::endl;

                            if (pair_stream >> value) {
                                // std::cout << "Value: " << value << std::endl;
                                if (key == "right_arm") {
                                    right_arm_angle = value;
                                    // std::cout << "right_arm_angle: " << right_arm_angle << std::endl;
                                } else if (key == "right_shoulder") {
                                    right_shoulder_arm_angle = value;
                                    // std::cout << "right_shoulder_arm_angle: " << right_shoulder_arm_angle << std::endl;
                                }
                            } else {
                                std::cerr << "Failed to parse value for key: " << key << std::endl;
                            }
                        } else {
                            std::cerr << "Failed to parse key-value pair: " << key_value_pair << std::endl;
                        }
                    }

                    // 构造 JSON 数据
                    json received_data;
                    received_data["shoulder_arm_angles"] = right_shoulder_arm_angle;
                    received_data["arm_angles"] = right_arm_angle;
                    // std::cout << "Received data:rot: " << received_data["shoulder_arm_angles"] << std::endl;

                    


                    // 提取数据
                    double roll_temp = received_data["shoulder_arm_angles"];
                    roll_temp = -(180-roll_temp);//-(180-theta1),-roll_temp在[0，180]，目标在[0,90]
                    
                    // 将 roll_temp 从 [-180,0] 映射到 [0, 90]
                    double roll_mapped = (roll_temp -(-180)) / (0-(-180)) * (90-0)+0;
                    double roll = roll_mapped * 3.14159 / 180; // 转换为弧度

                    double rot_temp = received_data["arm_angles"];
                    rot_temp = -(90-rot_temp);//-(90-theta2),-rot_temp在[-90，90]，目标在[0,90]
        
                    // 将 rot_temp 从 [-90, 90] 映射到 [0, 90]
                    double rot_mapped = (rot_temp -(-90)) / (90-(-90)) * (90-0)+0;
                    double rot = rot_mapped * 3.14159 / 180; // 转换为弧度

                    std::cout << "Roll (shoulder_arm_angles): " << roll
                              << ", Rot (arm_angles): " << rot << std::endl;
                    
                    while (true) {
                        double hor = 0.5;  // 横向移动，只能为正值
                        double ver = 0;    // 外伸，只能为负值
                        double linear_velocity = 40.0;  // 线速度
                        double rotational_speed = 40.0; // 旋转速度

                        arm.MoveToPosition(roll, hor, ver, rot, linear_velocity, rotational_speed);

                        double p[4] = {0.0, 0.0, 0.0, 0.0};
                        while (true) {
                            arm.getPosition(p);
                            std::cout << p[0] << " " << p[1] << " " << p[2] << " " << p[3] << std::endl;

                            // 检查误差是否在允许范围内
                            if (std::abs(p[0] - roll) <= 0.005 &&
                                std::abs(p[1] - hor) <= 0.005 &&
                                std::abs(p[2] - ver) <= 0.005 &&
                                std::abs(p[3] - rot) <= 0.005) {
                                std::cout << "目标位置已达到，准备接收下一条指令。" << std::endl;
                                break;
                            }

                            // 添加一个小的延迟，避免频繁查询机械臂位置
                            usleep(10000); // 延迟 10 毫秒
                        }
                    }

                } catch (const std::exception &e) {
                    std::cerr << "Error parsing data: " << e.what() << std::endl;
                }
                
            }
        } else if (valread == 0) {
            std::cout << "Client disconnected." << std::endl;
            arm.stop();
            break;
        } else {
            std::cerr << "Read error occurred." << std::endl;
            break;
        }
    }

    close(new_socket);
    close(server_fd);

    return 0;
}