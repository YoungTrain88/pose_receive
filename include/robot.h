#pragma once

namespace autopicker
{

    class Arm
    {
    private:
        std::string errorCode[4] = {"", "", "", ""};
        bool roll_arrived = false, hor_arrived = false, ver_arrived=false;
        bool isArrived[4] = {false, false, false, false};
        int pluse[4] = {0, 0, 0, 0};
        double original_pulses[4] = {206, -522854, -2378510, 65536}; // 0位的脉冲值
        char gripperStatus[9] = {' ', ' ', ' ', ' ', ' ',' ',' ',' ','\0'};
        bool logger = false;
    public:
        Arm();
        ~Arm();
        /**
         * @brief Initialize the arm with the node ids and gripper ids
         * @param nodeId_1 The node id of the first motor
         * @param nodeId_2 The node id of the second motor
         * @param nodeId_3 The node id of the third motor
         * @param nodeId_4 The node id of the fourth motor
         */
        void init(const int nodeId_1, const int nodeId_2, const int nodeId_3, const int nodeId_4, const int grippeId_1); 
        /**
         * @brief Move the arm to the specified position
         * @param position_1 The position of the first motor
         * @param position_2 The position of the second motor
         * @param position_3 The position of the third motor
         * @param position_4 The position of the fourth motor
         * @param speed_1 The speed of the first motor, the speed is in the range of 0 to 1023, 0 is the slowest and 1023 is the fastest, the default value is 10
         * @param speed_2 The speed of the second motor,the speed is in the range of 0 to 1023, 0 is the slowest and 1023 is the fastest, the default value is 10
         */
        bool MoveToPosition(double position_1, double position_2, double position_3, double position_4, double speed_1 = 10, int speed_2 = 10);
        /**
         * @brief Get the current position of the arm
         * @return The current position of the arm
         */
        
        void getPosition(double *res);
        /**
         * @brief Get the current speed of the arm
         * @return The current speed of the arm
         */
        std::string* getErrorCode();
        /**
         * @brief Open the gripper
         * @return True if the gripper is opened successfully, false otherwise
         */
        void open();
        /**
         * @brief Close the gripper
         * @return True if the gripper is closed successfully, false otherwise
         */
        void close();
        /**
         * @brief Rotate the arm clockwise
         * @return True if the arm is rotated successfully, false otherwise
         */
        void rotate_clockwise();
        /**
         * @brief Rotate the arm anticlockwise
         */
        void rotate_anticlockwise();
        /**
         * @brief extend the arm
         * @return True if the arm is extend successfully, false otherwise
         */
        void extend();
        /**
         * @brief retract the arm
         * @return True if the arm is retract successfully, false otherwise
         */
        void retract();
        
        /**
         * @brief Check if the gripper is open
         */
        bool isOpen();
        /**
         * @brief Check if the gripper is clockwise
         */
        bool isClockwise();

        /**
         * @brief Check if the arm is extended
         */
        bool isextend();
        
    };

} // namespace autopicker