#include <iostream>
#include <string>
#include <sstream>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <nlohmann/json.hpp> // 您的代码包含了此库，但未使用，保留
#include <thread>
#include <mutex>
#include <condition_variable> // 在新逻辑中不再需要，但保留以备将来使用
#include <chrono>             // 用于实现固定频率循环
#include <algorithm>          // 用于 std::max 和 std::min
#include "robot.h"

using json = nlohmann::json;

// --- 全局变量和同步机制 ---
// 使用互斥锁保护共享数据
std::mutex data_mutex;
// global_* 变量现在存储的是由网络线程提供的“原始目标位置”
double global_roll = 0.0, global_rot = 0.0, global_ver = 0.0, global_hor = 0.0;


/**
 * @brief 将一个值从一个范围线性映射到另一个范围。
 * @param value 要映射的输入值。
 * @param fromLow 输入范围的下限。
 * @param fromHigh 输入范围的上限。
 * @param toLow 输出范围的下限。
 * @param toHigh 输出范围的上限。
 * @return 映射后的值。
 *
 * @note 这是解决“不连续映射”问题的关键。它能确保输入值的微小变化
 * 只会导致输出值的微小变化，避免了目标点的突变。
 */
double mapValue(double value, double fromLow, double fromHigh, double toLow, double toHigh) {
    // 首先将输入值限制在输入范围内，防止超出范围导致异常映射
    value = std::max(fromLow, std::min(value, fromHigh));
    // 执行线性映射计算
    return toLow + (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow);
}


/**
 * @brief 机械臂控制线程函数（优化版）
 * @param arm 机械臂对象的引用。
 * * @details
 * 该函数采用两种关键优化来确保平滑运动：
 * 1.  **固定频率控制循环**: 以恒定的频率（例如50Hz）运行，而不是被动地等待新数据。
 * 这确保了控制指令以稳定的速率发送给机械臂，避免了因网络延迟抖动导致的控制不均。
 * 2.  **低通滤波器 (EMA)**: 不直接使用接收到的目标位置，而是将其作为“期望值”。
 * 实际发送给机械臂的目标位置是经过指数移动平均（Exponential Moving Average）滤波后的值。
 * 这会使目标点平滑地“追赶”期望值，就像给系统增加了惯性，从而消除抖动。
 */
void controlArm(autopicker::Arm &arm) {
    // 机械臂运动参数
    double linear_velocity = 250.0;
    double rotational_speed = 150.0;

    // --- 平滑控制参数 ---
    // 平滑因子 (alpha)，取值在 0 到 1 之间。值越小，运动越平滑，但响应越慢。
    // 这是一个需要根据实际效果反复调试的关键参数。建议从 0.05 开始尝试。
    const double alpha = 0.05; 
    
    // 控制循环频率 (50Hz)，即每 20 毫秒循环一次
    const auto loop_period = std::chrono::milliseconds(20);

    // 存储经过平滑滤波后的目标位置，这些值将直接发送给机械臂
    double smoothed_roll = 0.0, smoothed_rot = 0.0, smoothed_ver = 0.0, smoothed_hor = 0.0;
    bool is_first_run = true; // 用于在首次运行时初始化平滑位置

    // --- 主控制循环 ---
    while (true) {
        auto loop_start_time = std::chrono::steady_clock::now();

        // 步骤 1: 从全局变量中获取最新的“原始目标位置”
        double target_roll, target_rot, target_ver, target_hor;
        {
            std::lock_guard<std::mutex> lock(data_mutex);
            target_roll = global_roll;
            target_rot = global_rot;
            target_ver = global_ver;
            target_hor = global_hor;
        }

        // 步骤 2: 首次运行时，直接将平滑位置设置为目标位置，避免从零点突变
        if (is_first_run) {
            smoothed_roll = target_roll;
            smoothed_rot = target_rot;
            smoothed_ver = target_ver;
            smoothed_hor = target_hor;
            is_first_run = false;
        }

        // 步骤 3: 应用指数移动平均滤波，平滑地更新目标
        // 公式: new_smoothed_value = alpha * new_target + (1 - alpha) * old_smoothed_value
        smoothed_roll = alpha * target_roll + (1.0 - alpha) * smoothed_roll;
        smoothed_rot  = alpha * target_rot  + (1.0 - alpha) * smoothed_rot;
        smoothed_ver  = alpha * target_ver  + (1.0 - alpha) * smoothed_ver;
        smoothed_hor  = alpha * target_hor  + (1.0 - alpha) * smoothed_hor;

        // 步骤 4: 使用平滑后的值来控制机械臂
        arm.MoveToPosition(smoothed_roll, smoothed_hor, smoothed_ver, smoothed_rot, linear_velocity, rotational_speed);
        
        // 步骤 5: 等待直到下一个循环周期的开始，以维持固定的循环频率
        std::this_thread::sleep_until(loop_start_time + loop_period);
    }
}

int main() {
    autopicker::Arm arm = autopicker::Arm();
    arm.init(1, 2, 3, 4, 5);

    int server_fd, new_socket;
    struct sockaddr_in address;
    int addrlen = sizeof(address);
    char buffer[2048] = {0};

    server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd == 0) {
        std::cerr << "Socket creation failed!" << std::endl;
        return -1;
    }

    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(12345);

    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0) {
        std::cerr << "Socket bind failed!" << std::endl;
        return -1;
    }

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

    // --- 网络数据接收与解析循环 ---
    while (true) {
        int valread = read(new_socket, buffer, 1024);
        if (valread > 0) {
            buffer[valread] = '\0';
            std::string data(buffer);
            std::istringstream ss(data);
            std::string line;
            
            while (std::getline(ss, line)) {
                try {
                    double right_arm_angle = 0.0, right_shoulder_arm_angle = 0.0;
                    double left_arm_angle = 0.0, left_shoulder_arm_angle = 0.0;
                    
                    // (您的解析逻辑保持不变)
                    std::istringstream line_stream(line);
                    std::string key_value_pair;
                    while (std::getline(line_stream, key_value_pair, ',')) {
                        std::istringstream pair_stream(key_value_pair);
                        std::string key;
                        double value;
                        if (std::getline(pair_stream, key, ':')) {
                            if (!key.empty() && key.front() == '"') key.erase(0, 1);
                            if (!key.empty() && key.back() == '"') key.pop_back();

                            if (pair_stream >> value) {
                                if (key == "right_arm") right_arm_angle = value;
                                else if (key == "right_shoulder") right_shoulder_arm_angle = value;
                                else if (key == "left_arm") left_arm_angle = value;
                                else if (key == "left_shoulder") left_shoulder_arm_angle = value;
                            }
                        }
                    }

                    // --- 【优化】使用连续的线性映射计算目标值 ---
                    // 您需要根据实际需求定义好输入和输出的范围
                    // 这里的范围仅为示例，请务必根据您的机械臂和期望效果进行调整！
                    const double PI = 3.1415926535;

                    // 示例: 将右肩角度 [0, 180] 映射到 roll [PI, 0] (180度到0度)
                    double roll_temp = mapValue(right_shoulder_arm_angle, 0.0, 180.0, 180.0, 0.0);
                    double roll = roll_temp * PI / 180.0;

                    // 示例: 将右臂角度 [0, 180] 映射到 rot [0, PI/2] (0到90度)
                    double rot = mapValue(right_arm_angle, 0.0, 180.0, 0.0, PI / 2.0);

                    // 示例: 将左臂角度 [90, 180] 映射到 ver [-0.1, -0.4]
                    double ver = mapValue(left_arm_angle, 90.0, 180.0, -0.1, -0.4);

                    // 示例: 将左肩角度 [0, 90] 映射到 hor [0, 1.0]
                    double hor = mapValue(left_shoulder_arm_angle, 0.0, 90.0, 0.0, 1.0);

                    // std::cout << "Raw Targets -> Roll: " << roll << ", Rot: " << rot << ", Ver: " << ver << ", Hor: " << hor << std::endl;

                    // --- 更新全局变量 ---
                    {
                        std::lock_guard<std::mutex> lock(data_mutex);
                        global_roll = roll;
                        global_rot = rot;
                        global_ver = ver;
                        global_hor = hor;
                    }
                    // 注意：在新模式下，不再需要 new_data_available 和 data_cv.notify_one()

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
