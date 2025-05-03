#include <iostream>
#include <string>
#include <sstream>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <nlohmann/json.hpp> // JSON 库
#include <thread>
#include <mutex>
#include <condition_variable>
#include "robot.h"

using json = nlohmann::json;

// 全局变量和同步机制
std::mutex data_mutex;
std::condition_variable data_cv;
double global_roll = 0.0, global_rot = 0.0;
bool new_data_available = false;

void controlArm(autopicker::Arm &arm) {
    double hor = 0.5;  // 横向移动，只能为正值
    double ver = 0;    // 外伸，只能为负值
    double linear_velocity = 40.0;  // 线速度
    double rotational_speed = 40.0; // 旋转速度

    while (true) {
        std::unique_lock<std::mutex> lock(data_mutex);
        data_cv.wait(lock, [] { return new_data_available; });

        // 获取最新的目标位置
        double roll = global_roll;
        double rot = global_rot;
        new_data_available = false;
        lock.unlock();

        // 控制机械臂
        arm.MoveToPosition(roll, hor, ver, rot, linear_velocity, rotational_speed);
    }
}

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

    // 启动机械臂控制线程
    std::thread control_thread(controlArm, std::ref(arm));

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
                            }
                        }
                    }

                    // // 计算 roll 和 rot
                    // double roll_temp = -(180 - right_shoulder_arm_angle);
                    // double roll_mapped = (roll_temp - (-180)) / (0 - (-180)) * (90 - 0) + 0;
                    // double roll = roll_mapped * 3.14159 / 180;
                    double roll_temp = (right_shoulder_arm_angle - 90) / (180 - 90) * 1.5;    
                    double roll = roll_temp ;// * 3.14159 / 180;


                    double rot_temp = -(90 - right_arm_angle);
                    double rot_mapped = (rot_temp - (-90)) / (90 - (-90)) * (90 - 0) + 0;
                    double rot = rot_mapped * 3.14159 / 180;

                    std::cout << "Roll (shoulder_arm_angles): " << roll
                              << ", Rot (arm_angles): " << rot << std::endl;

                    // 更新全局变量
                    {
                        std::lock_guard<std::mutex> lock(data_mutex);
                        global_roll = roll;
                        global_rot = rot;
                        new_data_available = true;
                    }
                    data_cv.notify_one();

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

    control_thread.join();
    close(new_socket);
    close(server_fd);

    return 0;
}