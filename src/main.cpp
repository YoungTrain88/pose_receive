#include <iostream>
#include <string>
#include <sstream>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>

int main() {
    int server_fd, new_socket;
    struct sockaddr_in address;
    int addrlen = sizeof(address);
    char buffer[1024] = {0};

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
            while (std::getline(ss, line)) {
                try {
                    // 解析数据，假设格式为 "right_arm:0.0,right_shoulder:0.0"
                    std::istringstream line_stream(line);
                    std::string key_value_pair;
                    double right_arm_angle = 0.0, right_shoulder_arm_angle = 0.0;

                    while (std::getline(line_stream, key_value_pair, ',')) {
                        std::istringstream pair_stream(key_value_pair);
                        std::string key;
                        double value;

                        if (std::getline(pair_stream, key, ':') && pair_stream >> value) {
                            if (key == "right_arm") {
                                right_arm_angle = value;
                            } else if (key == "right_shoulder") {
                                right_shoulder_arm_angle = value;
                            }
                        } else {
                            std::cerr << "Invalid key-value pair: " << key_value_pair << std::endl;
                        }
                    }

                    std::cout << "Right Arm Angle: " << right_arm_angle
                              << ", Right Shoulder Arm Angle: " << right_shoulder_arm_angle << std::endl;
                } catch (const std::exception &e) {
                    std::cerr << "Error parsing data: " << e.what() << std::endl;
                }
            }
        } else if (valread == 0) {
            std::cout << "Client disconnected." << std::endl;
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