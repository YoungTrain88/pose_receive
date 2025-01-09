// robot.cpp

#include <iostream>
#include <robot.h>
#include <controlcan.h>
#include <vector>
#include <cmath>
#include <thread>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <bitset>
#include <unistd.h>
#include <mutex>
#include <cstdio>
#include <string>
#include <cstring>

using namespace autopicker;

long STEPS_PER_RAD_VALUE_kinco = 7864320 / 360;
long STEPS_PER_RAD_VALUE_tech = 131072 / 360;
long STEPS_PER_MM_VALUE = 327680 / 140;
int count = 0; // 数据列表中，用来存储列表序号。
std::mutex mtx;

Arm::Arm()
{
	// Constructor
}

Arm::~Arm()
{
	// Destructor
	VCI_ResetCAN(VCI_USBCAN2, 0, 0); // 复位CAN1通道。
	usleep(100000);					 // 延时100ms。
	VCI_ResetCAN(VCI_USBCAN2, 0, 1); // 复位CAN2通道。
	usleep(100000);					 // 延时100ms。
	VCI_CloseDevice(VCI_USBCAN2, 0); // 关闭设备。
}

long rotate_kinco(double radians)
{
	double degree = radians * 180.0 / 3.1415926;
	// std::cout << "kinco degree: " << degree << std::endl;
	long pluse = std::round(degree * STEPS_PER_RAD_VALUE_kinco);
	// std::cout << "pluse: " << pluse << std::endl;
	return pluse;
}

double rotate_kinco_position(long pluse)
{
	// std::cout << "kinco degree: " << degree << std::endl;
	double degree = pluse / STEPS_PER_RAD_VALUE_kinco;
	double radians = degree * 3.1415926 / 180.0;
	// std::cout << "pluse: " << pluse << std::endl;
	return radians;
}

long rotate_tech(double radians)
{
	double degree = radians * 180.0 / 3.1415926;
	// std::cout << "tech degree: " << degree << std::endl;
	long pluse = std::round(degree * STEPS_PER_RAD_VALUE_tech);
	return pluse;
}

double rotate_tech_position(long pluse)
{
	double degree = pluse / STEPS_PER_RAD_VALUE_tech;
	// std::cout << "tech degree: " << degree << std::endl;
	double radians = degree * 3.1415926 / 180.0;
	return radians;
}

std::vector<long> corexy_joint(double delta_x, double delta_y)
{
	double delta_a = (delta_x + delta_y) * STEPS_PER_MM_VALUE;
	double delta_b = (delta_x - delta_y) * STEPS_PER_MM_VALUE;

	long motor2_pulses = round(delta_a);
	long motor3_pulses = round(delta_b);

	// std::cout << "2号电机 pulses: {motor2_pulses} (方向 {'counterclockwise' if motor2_pulses > 0 else 'clockwise'})" << std::endl;
	// std::cout << "3号电机 pulses: {motor3_pulses} (方向 {'counterclockwise' if motor3_pulses > 0 else 'clockwise'})" << std::endl;

	std::vector<long> res;
	res.push_back(motor2_pulses);
	res.push_back(motor3_pulses);
	return res;
}

std::vector<double> corexy_joint_position(long delta_a, long delta_b)
{
	double delta_x = (delta_a + delta_b) / (2 * STEPS_PER_MM_VALUE);
	double delta_y = (delta_a - delta_b) / (2 * STEPS_PER_MM_VALUE);

	// std::cout << "2号电机 pulses: {motor2_pulses} (方向 {'counterclockwise' if motor2_pulses > 0 else 'clockwise'})" << std::endl;
	// std::cout << "3号电机 pulses: {motor3_pulses} (方向 {'counterclockwise' if motor3_pulses > 0 else 'clockwise'})" << std::endl;

	std::vector<double> res;
	res.push_back(delta_x);
	res.push_back(delta_y);
	return res;
}

/**
 * @brief 将字节数组转换为二进制字符串
 */
std::string byteArrayToBinaryString(const std::vector<uint8_t> &byteArray)
{
	std::ostringstream oss;
	for (const auto &byte : byteArray)
	{
		oss << std::bitset<8>(byte);
	}
	return oss.str();
}

/**
 * @brief 将二进制字符串转换为十六进制字符串
 */
std::string binaryStringToHexString(const std::string &binaryString)
{
	std::stringstream ss;
	for (size_t i = 0; i < binaryString.size(); i += 4)
	{
		std::bitset<4> bits(binaryString.substr(i, 4));
		ss << std::hex << bits.to_ulong();
	}
	return ss.str();
}

/**
 * @brief 将字节数组转换为十进制
 */
int byteArrayToDecimalInteger(const std::vector<uint8_t> &byteArray)
{
	int decimalValue = 0;
	for (size_t i = 0; i < byteArray.size(); ++i)
	{
		decimalValue = (decimalValue << 8) | byteArray[i];
	}
	return decimalValue;
}

std::string byteArrayToHexString(const std::vector<uint8_t> &byteArray)
{
	std::ostringstream oss;
	for (const auto &byte : byteArray)
	{
		oss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte);
	}
	return oss.str();
}

void isErrorFunc(VCI_CAN_OBJ rec, std::string *errorCode)
{
	// if (rec.ExternFlag == 0)
	// 	printf(" Standard "); // 帧格式：标准帧
	// if (rec.ExternFlag == 1)
	// 	printf(" Extend   "); // 帧格式：扩展帧
	// if (rec.RemoteFlag == 0)
	// 	printf(" Data   "); // 帧类型：数据帧
	// if (rec.RemoteFlag == 1)
	// 	printf(" Remote "); // 帧类型：远程帧
	// printf("DLC:0x%02X", rec.DataLen); // 帧长度printf(" data:0x");	//数据
	// printf("\n recData[0]:%02X\n", rec.Data[0]);
	//  判断是否是6043对象
	if (rec.Data[0] == 0x4B && rec.Data[1] == 0x3F && rec.Data[2] == 0x60 && rec.Data[3] == 0x00)
	{
		std::vector<uint8_t> byteArray = {rec.Data[5], rec.Data[4]};
		std::string hexString = byteArrayToHexString(byteArray);
		std::string errorCodeStr = std::to_string(rec.ID - 0x580 - 1) + ":" + hexString;
		errorCode[rec.ID - 0x580 - 1] = errorCodeStr.c_str();
		std::cout << hexString << std::endl;
	}
}

/**
 * @brief 根据消息内容获取各个电机的位置信息
 */
void isPositionFunc(VCI_CAN_OBJ rec, int *pluse)
{
	// if (rec.ExternFlag == 0)
	// 	printf(" Standard "); // 帧格式：标准帧
	// if (rec.ExternFlag == 1)
	// 	printf(" Extend   "); // 帧格式：扩展帧
	// if (rec.RemoteFlag == 0)
	// 	printf(" Data   "); // 帧类型：数据帧
	// if (rec.RemoteFlag == 1)
	// 	printf(" Remote "); // 帧类型：远程帧
	// printf("DLC:0x%02X", rec.DataLen); // 帧长度printf(" data:0x");	//数据
	// printf("\n recData[0]:%02X\n", rec.Data[0]);
	//  判断是否是6043对象
	if (rec.Data[0] == 0x43 && rec.Data[1] == 0x64 && rec.Data[2] == 0x60 && rec.Data[3] == 0x00)
	{
		std::vector<uint8_t> byteArray = {rec.Data[7], rec.Data[6], rec.Data[5], rec.Data[4]};
		int decimalString = byteArrayToDecimalInteger(byteArray);
		pluse[rec.ID - 0x580 - 1] = decimalString;
		// printf("position ID:%d %d   \n", (rec.ID - 0x580 - 1), decimalString); // ID
		//  printf("RX ID:0x%08X   Binary String:%s", rec.ID, binaryString[10]); // ID
		//  std::cout << byteArray[0] << " " << byteArray[1] << " " << byteArray[2] << " " << byteArray[3] << std::endl;
	}
}

/**
 * @brief 判断各个臂是否到达目标位置
 */
void isArrivedFunc(VCI_CAN_OBJ rec, bool *isArrived)
{

	// if (rec.ExternFlag == 0)
	// 	printf(" Standard "); // 帧格式：标准帧
	// if (rec.ExternFlag == 1)
	// 	printf(" Extend   "); // 帧格式：扩展帧
	// if (rec.RemoteFlag == 0)
	// 	printf(" Data   "); // 帧类型：数据帧
	// if (rec.RemoteFlag == 1)
	// 	printf(" Remote "); // 帧类型：远程帧
	// printf("Arrived_func   DLC:0x%02X ", rec.DataLen); // 帧长度printf(" data:0x");	//数据
	// printf("\n recData[0]:%02X\n", rec.Data[0]);
	//  判断是否是6041对象
	if (rec.Data[0] == 0x4B && rec.Data[1] == 0x41 && rec.Data[2] == 0x60 && rec.Data[3] == 0x00)
	{
		std::vector<uint8_t> byteArray = {rec.Data[5], rec.Data[4]};
		std::string binaryString = byteArrayToBinaryString(byteArray);
		// std::cout << binaryString << std::endl;
		// printf("arrived ID:0x%08X %d \n", rec.ID, binaryString[5]); // ID
		if (binaryString[11] == '1')
		{ // 从后往前数第10位为1表示到达目标位置
			mtx.lock();
			isArrived[rec.ID - 0x580 - 1] = true;
			// std::cout << "arrived ID:" << (rec.ID - 0x580) << std::endl;
			mtx.unlock();
		}
		else
		{
			mtx.lock();
			isArrived[rec.ID - 0x580 - 1] = false;
			mtx.unlock();
		}
		// printf("RX ID:0x%08X   Binary String:%s", rec.ID, binaryString[10]); // ID
		// std::cout << byteArray[0] << " " << byteArray[1] << " " << byteArray[2] << " " << byteArray[3] << std::endl;
	}

	// std::cout << "\n end Arrived_func \n " << std::endl;
	//  std::cout << " id:" << (rec.ID - 0x580) << std::endl;
	//  printf(" TimeStamp:0x%08X", rec.TimeStamp); // 时间标识
}

void isGripperFunc(VCI_CAN_OBJ rec, char *gripperStatus)
{
	if (rec.ID == 0x207)
	{ // 手爪反馈信息
		// printf("GripperFunc   DLC:0x%02X ", rec.DataLen); // 帧长度printf(" data:0x");	//数据
		// printf("\n recData[0]:%02X\n", rec.Data[0]);
		std::vector<uint8_t> byteArray = {rec.Data[0]};
		std::string binaryString = byteArrayToBinaryString(byteArray);
		// std::cout << binaryString << std::endl;
		// std::cout << binaryString.size() << std::endl;
		strcpy(gripperStatus, binaryString.c_str());
		// std::cout << gripperStatus << std::endl;
	}
}

void *receive_func(bool *isArrived, int *pluse, std::string *errorCode, char *gripperStatus, bool logger) // 接收线程
{
	// std::cout << "receive_func" << std::endl;
	int reclen = 1;
	VCI_CAN_OBJ rec[3000]; // 接收缓存，设为3000为佳。
	int i, j;
	// int param = 1;
	// int *run = (int *)param; // 线程启动，退出控制。
	// std::cout << run << std::endl;
	int ind = 0;
	while (true) // 循环接收数据
	{
		// std::cout << "run thread" << std::endl;

		if ((reclen = VCI_Receive(VCI_USBCAN2, 0, ind, rec, 3000, 0)) >= 0) // 调用接收函数，如果有数据，进行数据处理显示。
		{
			// std::cout << "reclen:" << reclen << std::endl;
			for (j = 0; j < reclen; j++)
			{
				if (logger)
				{
					printf("Index:%04d  ", count);
					count++;										  // 序号递增
					printf("CAN%d RX ID:0x%08X", ind + 1, rec[j].ID); // ID
					if (rec[j].ExternFlag == 0)
						printf(" Standard "); // 帧格式：标准帧
					if (rec[j].ExternFlag == 1)
						printf(" Extend   "); // 帧格式：扩展帧
					if (rec[j].RemoteFlag == 0)
						printf(" Data   "); // 帧类型：数据帧
					if (rec[j].RemoteFlag == 1)
						printf(" Remote ");				   // 帧类型：远程帧
					printf("DLC:0x%02X ", rec[j].DataLen); // 帧长度printf(" data:0x");	//数据
					for (i = 0; i < rec[j].DataLen; i++)
					{
						printf(" %02X", rec[j].Data[i]);
					}
					printf(" TimeStamp:0x%08X \n", rec[j].TimeStamp); // 时间标识。
				}
				std::thread isArrivedThread(isArrivedFunc, rec[j], isArrived); // 创建接收新位置的线程
				isArrivedThread.detach();

				std::thread isPositionThread(isPositionFunc, rec[j], pluse); // 创建接收新位置的线程
				isPositionThread.detach();

				std::thread isErrorThread(isErrorFunc, rec[j], errorCode); // 创建接收新位置的线程
				isErrorThread.detach();

				std::thread isGripper(isGripperFunc, rec[j], gripperStatus); // 创建接收新位置的线程
				isGripper.detach();
			}
		}

		sleep(0.5);
		// ind = !ind;
	} // 变换通道号，以便下次读取另一通道，交替读取。
	printf("run thread exit\n"); // 退出接收线程
	pthread_exit(0);
}

void Arm::init(const int nodeId_1, const int nodeId_2, const int nodeId_3, const int nodeId_4, const int grippeId_1)
{
	// Initialize the arm with the node ids and gripper ids
	VCI_BOARD_INFO pInfo1[50];
	int num = VCI_FindUsbDevice2(pInfo1);
	std::cout << ">>USBCAN device number: " << num << std::endl;
	for (int i = 0; i < num; i++)
	{
		std::cout << "Serial_Num:" << pInfo1[i].str_Serial_Num << std::endl;
		std::cout << "hw_Type:" << pInfo1[i].str_hw_Type << std::endl;
		std::cout << "Firmware Version:" << pInfo1[i].fw_Version << std::endl;
	}
	if (VCI_OpenDevice(VCI_USBCAN2, 0, 0) == 1) // 是否打开设备
	{
		printf(">>open deivce success!\n"); // 打开设备成功
	}
	else
	{
		printf(">>open deivce error!\n");
		exit(1);
	}
	VCI_BOARD_INFO pInfo;
	if (VCI_ReadBoardInfo(VCI_USBCAN2, 0, &pInfo) == 1) // 读取设备序列号、版本等信息。
	{
		printf(">>Get VCI_ReadBoardInfo success!\n");
		std::cout << "Serial_Num:" << pInfo.str_Serial_Num << std::endl;
		std::cout << "hw_Type:" << pInfo.str_hw_Type << std::endl;
		std::cout << "Firmware Version:" << pInfo.fw_Version << std::endl;
	}
	else
	{
		printf(">>Get VCI_ReadBoardInfo error!\n");
		exit(1);
	}

	// 初始化参数，严格参数二次开发函数库说明书。  主程序从这这里开始
	VCI_INIT_CONFIG config;
	config.AccCode = 0;
	config.AccMask = 0xFFFFFFFF;
	config.Filter = 1;	   // 接收所有帧
	config.Timing0 = 0x00; /*设置波特率为500Kbps*/
	config.Timing1 = 0x1C;
	config.Mode = 0; // 正常模式
	if (VCI_InitCAN(VCI_USBCAN2, 0, 0, &config) != 1)
	{
		printf(">>Init CAN1 error\n");
		VCI_CloseDevice(VCI_USBCAN2, 0);
	}
	if (VCI_StartCAN(VCI_USBCAN2, 0, 0) != 1)
	{
		printf(">>Start CAN1 error\n");
		VCI_CloseDevice(VCI_USBCAN2, 0);
	}
	if (VCI_InitCAN(VCI_USBCAN2, 0, 1, &config) != 1)
	{
		printf(">>Init can2 error\n");
		VCI_CloseDevice(VCI_USBCAN2, 0);
	}
	if (VCI_StartCAN(VCI_USBCAN2, 0, 1) != 1)
	{
		printf(">>Start can2 error\n");
		VCI_CloseDevice(VCI_USBCAN2, 0);
	}
	std::cout << "init success" << std::endl;
	sleep(1);
	std::thread receivingThread(receive_func, isArrived, pluse, errorCode, gripperStatus, logger); // 创建接收新位置的线程
	receivingThread.detach();

	VCI_CAN_OBJ send[1];
	send[0].ID = 0;
	send[0].SendType = 0;
	send[0].RemoteFlag = 0;
	send[0].ExternFlag = 0;
	send[0].DataLen = 8;
	send[0].ID = 0X604;

	send[0].Data[0] = 0X2B;
	send[0].Data[1] = 0X40;
	send[0].Data[2] = 0X60;
	send[0].Data[3] = 0X00;
	send[0].Data[4] = 0X06;
	send[0].Data[5] = 0X00;
	send[0].Data[6] = 0X00;
	send[0].Data[7] = 0X00;
	VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1) == 1;

	send[0].ID = 0X604;
	send[0].Data[0] = 0X2F;
	send[0].Data[1] = 0X60;
	send[0].Data[2] = 0X60;
	send[0].Data[3] = 0X00;
	send[0].Data[4] = 0X01;
	send[0].Data[5] = 0X00;
	send[0].Data[6] = 0X00;
	send[0].Data[7] = 0X00;
	VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1) == 1;

	send[0].ID = 0X604;
	send[0].Data[0] = 0X2B;
	send[0].Data[1] = 0X60;
	send[0].Data[2] = 0X60;
	send[0].Data[3] = 0X00;
	send[0].Data[4] = 0X3F;
	send[0].Data[5] = 0X00;
	send[0].Data[6] = 0X00;
	send[0].Data[7] = 0X00;
	VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1);
}

bool Arm::MoveToPosition(double roll, double hor, double ver, double rot, double linear_velocity, int rotational_speed)
{
	// 需要发送的帧，结构体设置
	VCI_CAN_OBJ send[1];
	send[0].ID = 0;
	send[0].SendType = 0;
	send[0].RemoteFlag = 0;
	send[0].ExternFlag = 0;
	send[0].DataLen = 8;
	// 控制字
	for (size_t i = 0; i < 3; i++)
	{
		send[0].ID = 0X601 + i;
		send[0].Data[0] = 0X2B;
		send[0].Data[1] = 0X40;
		send[0].Data[2] = 0X60;
		send[0].Data[3] = 0X00;
		send[0].Data[4] = 0X2F;
		send[0].Data[5] = 0X00;
		send[0].Data[6] = 0X00;
		send[0].Data[7] = 0X00;
		VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1) == 1;
	}

	// 控制模式：位置模式
	for (size_t i = 0; i < 3; i++)
	{
		send[0].ID = 0X601 + i;
		send[0].Data[0] = 0X2F;
		send[0].Data[1] = 0X60;
		send[0].Data[2] = 0X60;
		send[0].Data[3] = 0X00;
		send[0].Data[4] = 0X01;
		send[0].Data[5] = 0X00;
		send[0].Data[6] = 0X00;
		send[0].Data[7] = 0X00;
		VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1) == 1;
	}

	double x = hor * 1000.0;
	double y = ver * 1000.0;
	double delta_pluse1 = rotate_kinco(roll);
	std::vector<long> pluses = corexy_joint(x, y);
	double delta_pluse2 = pluses[0];
	double detla_pluse3 = pluses[1];

	// double original_pulses[4] = {-8733, -499398, -534314, 65536}; // 0位的脉冲值
	long target_pluse1 = original_pulses[0] + delta_pluse1;
	long target_pluse2 = original_pulses[1] + delta_pluse2;
	long target_pluse3 = original_pulses[2] + detla_pluse3;
	long target_pluse4 = original_pulses[3] + rotate_tech(rot);

	if (logger)
	{
		std::cout << "target_pluse1:" << target_pluse1 << std::endl;
		std::cout << "target_pluse2:" << target_pluse2 << std::endl;
		std::cout << "target_pluse3:" << target_pluse3 << std::endl;
		std::cout << "target_pluse4:" << target_pluse4 << std::endl;
	}

	// 目标位置
	send[0].ID = 0X601 + 0;
	send[0].Data[0] = 0X23;
	send[0].Data[1] = 0X7A;
	send[0].Data[2] = 0X60;
	send[0].Data[3] = 0X00;
	send[0].Data[7] = (target_pluse1 >> 24) & 0xFF;
	send[0].Data[6] = (target_pluse1 >> 16) & 0xFF;
	send[0].Data[5] = (target_pluse1 >> 8) & 0xFF; // 23 7A  60 00 BF 44 04 00
	send[0].Data[4] = target_pluse1 & 0xFF;
	VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1) == 1;

	send[0].ID = 0X601 + 1;
	send[0].Data[0] = 0X23;
	send[0].Data[1] = 0X7A;
	send[0].Data[2] = 0X60;
	send[0].Data[3] = 0X00;
	send[0].Data[7] = (target_pluse2 >> 24) & 0xFF;
	send[0].Data[6] = (target_pluse2 >> 16) & 0xFF;
	send[0].Data[5] = (target_pluse2 >> 8) & 0xFF; // 23 7A  60 00 BF 44 04 00
	send[0].Data[4] = target_pluse1 & 0xFF;
	VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1) == 1;

	send[0].ID = 0X601 + 2;
	send[0].Data[0] = 0X23;
	send[0].Data[1] = 0X7A;
	send[0].Data[2] = 0X60;
	send[0].Data[3] = 0X00;
	send[0].Data[7] = (target_pluse3 >> 24) & 0xFF;
	send[0].Data[6] = (target_pluse3 >> 16) & 0xFF;
	send[0].Data[5] = (target_pluse3 >> 8) & 0xFF; // 23 7A  60 00 BF 44 04 00
	send[0].Data[4] = target_pluse1 & 0xFF;
	VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1) == 1;

	send[0].ID = 0X601 + 3;
	send[0].Data[0] = 0X23;
	send[0].Data[1] = 0X7A;
	send[0].Data[2] = 0X60;
	send[0].Data[3] = 0X00;
	send[0].Data[7] = (target_pluse4 >> 24) & 0xFF;
	send[0].Data[6] = (target_pluse4 >> 16) & 0xFF;
	send[0].Data[5] = (target_pluse4 >> 8) & 0xFF; // 23 7A  60 00 BF 44 04 00
	send[0].Data[4] = target_pluse4 & 0xFF;
	VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1) == 1;

	int64_t velocity1 = (int64_t)(linear_velocity * 512 * 65536 / 1875); // 电机1速度
	// std::cout << "velocity1:" << velocity1 << std::endl;

	for (size_t i = 0; i < 3; i++)
	{
		// 梯形速度
		send[0].ID = 0X601 + i;
		send[0].Data[0] = 0X23;
		send[0].Data[1] = 0X81;
		send[0].Data[2] = 0X60;
		send[0].Data[3] = 0X00;
		// send[0].Data[7] = 0X00;
		// send[0].Data[6] = 0X05;
		// send[0].Data[5] = 0X76;
		// send[0].Data[4] = 0X19;
		send[0].Data[7] = (velocity1 >> 24) & 0xFF;
		send[0].Data[6] = (velocity1 >> 16) & 0xFF;
		send[0].Data[5] = (velocity1 >> 8) & 0xFF;
		send[0].Data[4] = velocity1 & 0xFF;
		VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1) == 1;
	}

	// 泰科的梯形速度
	send[0].ID = 0X601 + 3;
	send[0].Data[0] = 0X23;
	send[0].Data[1] = 0X81;
	send[0].Data[2] = 0X60;
	send[0].Data[3] = 0X00;
	send[0].Data[7] = 0X00;
	send[0].Data[6] = 0X05;
	send[0].Data[5] = 0X76;
	send[0].Data[4] = 0X19;
	// send[0].Data[7] = (velocity1 >> 24) & 0xFF;
	// send[0].Data[6] = (velocity1 >> 16) & 0xFF;
	// send[0].Data[5] = (velocity1 >> 8) & 0xFF;
	// send[0].Data[4] = velocity1 & 0xFF;
	VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1) == 1;

	for (size_t i = 0; i < 3; i++)
	{
		// 控制字
		send[0].ID = 0X601 + i;
		send[0].Data[0] = 0X2B;
		send[0].Data[1] = 0X40;
		send[0].Data[2] = 0X60;
		send[0].Data[3] = 0X00;
		send[0].Data[4] = 0X3F;
		send[0].Data[5] = 0X00;
		send[0].Data[6] = 0X00;
		send[0].Data[7] = 0X00;
		VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1) == 1;
	}

	send[0].ID = 0X604;
	send[0].Data[0] = 0X2B;
	send[0].Data[1] = 0X40;
	send[0].Data[2] = 0X60;
	send[0].Data[3] = 0X00;
	send[0].Data[4] = 0X0F;
	send[0].Data[5] = 0X00;
	send[0].Data[6] = 0X00;
	send[0].Data[7] = 0X00;
	VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1);
	send[0].ID = 0X604;
	send[0].Data[0] = 0X2B;
	send[0].Data[1] = 0X40;
	send[0].Data[2] = 0X60;
	send[0].Data[3] = 0X00;
	send[0].Data[4] = 0X3F;
	send[0].Data[5] = 0X00;
	send[0].Data[6] = 0X00;
	send[0].Data[7] = 0X00;
	VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1);

	sleep(1);
	mtx.lock();
	isArrived[0] = false;
	isArrived[1] = false;
	isArrived[2] = false;
	isArrived[3] = false;
	mtx.unlock();
	while (true) // 判断三个电机是否已经到达
	{
		if (isArrived[0] && isArrived[1] && isArrived[2] && isArrived[3])
		{
			if (logger)
				std::cout << "motors are arrived" << std::endl;
			break;
		}
		/* 发送查询命令 */
		for (size_t i = 1; i < 5; i++)
		{
			if (isArrived[i - 1] == false)
			{
				send[0].ID = 0X600 + i;
				send[0].Data[0] = 0X40;
				send[0].Data[1] = 0X41;
				send[0].Data[2] = 0X60;
				send[0].Data[3] = 0X00;
				send[0].Data[4] = 0X00;
				send[0].Data[5] = 0X00;
				send[0].Data[6] = 0X00;
				send[0].Data[7] = 0X00;
				VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1) == 1;
				sleep(0.5);
			}
		}
	}
}

std::string *Arm::getErrorCode()
{
	// Get the current position of the arm
	errorCode[0] = "";
	errorCode[1] = "";
	errorCode[2] = "";
	errorCode[3] = "";
	VCI_CAN_OBJ send[1];
	send[0].ID = 0;
	send[0].SendType = 0;
	send[0].RemoteFlag = 0;
	send[0].ExternFlag = 0;
	send[0].DataLen = 8;
	/* 发送查询命令 */
	for (size_t i = 1; i < 5; i++)
	{
		send[0].ID = 0X600 + i;
		send[0].Data[0] = 0X40;
		send[0].Data[1] = 0X3F;
		send[0].Data[2] = 0X60;
		send[0].Data[3] = 0X00;
		send[0].Data[4] = 0X00;
		send[0].Data[5] = 0X00;
		send[0].Data[6] = 0X00;
		send[0].Data[7] = 0X00;
		VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1) == 1;
		sleep(0.05);
	}
	while (true)
	{
		if (errorCode[0] != "" && errorCode[1] != "" && errorCode[2] != "" && errorCode[3] != "")
		{
			printf("position ID: %d  %d %d  \n", pluse[0], pluse[1], pluse[2]); // ID
			std::cout << pluse[0] << " " << pluse[1] << " " << pluse[2] << std::endl;
			return errorCode;
		}
	}
}

void Arm::getPosition(double *res)
{
	pluse[0] = -1;
	pluse[1] = -1;
	pluse[2] = -1;
	pluse[3] = -1;
	VCI_CAN_OBJ send[1];
	send[0].ID = 0;
	send[0].SendType = 0;
	send[0].RemoteFlag = 0;
	send[0].ExternFlag = 0;
	send[0].DataLen = 8;
	/* 发送查询命令 */
	for (size_t i = 1; i < 5; i++)
	{
		send[0].ID = 0X600 + i;
		send[0].Data[0] = 0X40;
		send[0].Data[1] = 0X64;
		send[0].Data[2] = 0X60;
		send[0].Data[3] = 0X00;
		send[0].Data[4] = 0X00;
		send[0].Data[5] = 0X00;
		send[0].Data[6] = 0X00;
		send[0].Data[7] = 0X00;
		VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1) == 1;
		sleep(0.05);
	}

	while (true)
	{
		res[0] = pluse[0];
		res[1] = pluse[1];
		res[2] = pluse[2];
		res[3] = pluse[3];
		// std::cout << "position ID: " << pluse[0] << " " << pluse[1] << " " << pluse[2] << " " << pluse[3] << std::endl;
		if (pluse[0] != -1 && pluse[1] != -1 && pluse[2] != -1 && pluse[3] != -1)
		{

			long delta_pluse1 = pluse[0] - original_pulses[0];
			long delta_pluse2 = pluse[1] - original_pulses[1];
			long detla_pluse3 = pluse[2] - original_pulses[2];
			long detla_pluse4 = pluse[3] - original_pulses[3];

			double roll = rotate_kinco_position(delta_pluse1);
			std::vector<double> xy = corexy_joint_position(delta_pluse2, detla_pluse3);
			double hor = xy[0] / 1000.0;
			double ver = xy[1] / 1000.0;
			double rot = rotate_tech_position(detla_pluse4);
			res[0] = roll;
			res[1] = hor;
			res[2] = ver;
			//std::cout << "getpluse: " << pluse[0] << " " << pluse[1] << " " << pluse[2] << " " << pluse[3] << std::endl;
			//std::cout << "roll:" << roll << " hor:" << hor << " ver:" << ver << " rot:" << rot << std::endl;
			res[3] = rot;
			break;
		}
	}
	// return res;
}

void Arm::open()
{
	VCI_CAN_OBJ send[1];
	send[0].SendType = 0;
	send[0].RemoteFlag = 0;
	send[0].ExternFlag = 0;
	send[0].DataLen = 1;
	send[0].ID = 0X200 + 7;
	send[0].Data[0] = 0X00;
	VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1) == 1;
	gripperStatus[1] = '*';
	while (gripperStatus[1] == '*')
	{
		// std::cout << "gripperStatus:" << gripperStatus << std::endl;
	}
	gripperStatus[7] = '1';
	std::string hexString = binaryStringToHexString(gripperStatus);
	std::cout << "hexString:" << hexString << std::endl;

	send[0].SendType = 0;
	send[0].RemoteFlag = 0;
	send[0].ExternFlag = 0;
	send[0].DataLen = 1;
	send[0].ID = 0X100 + 7;
	send[0].Data[0] = std::stoul(hexString, nullptr, 16);
	;
	VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1) == 1;
}

void Arm::close()
{
	VCI_CAN_OBJ send[1];
	send[0].SendType = 0;
	send[0].RemoteFlag = 0;
	send[0].ExternFlag = 0;
	send[0].DataLen = 1;
	send[0].ID = 0X200 + 7;
	send[0].Data[0] = 0X00;
	VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1) == 1;
	gripperStatus[1] = '*';
	while (gripperStatus[1] == '*')
	{
		// std::cout << "gripperStatus:" << gripperStatus << std::endl;
	}
	if (logger)
		std::cout << "back gripperStatus:" << gripperStatus << std::endl;
	gripperStatus[7] = '0';
	std::string hexString = binaryStringToHexString(gripperStatus);
	if (logger)
		std::cout << "hexString:" << hexString << std::endl;

	send[0].SendType = 0;
	send[0].RemoteFlag = 0;
	send[0].ExternFlag = 0;
	send[0].DataLen = 1;
	send[0].ID = 0X100 + 7;
	send[0].Data[0] = std::stoul(hexString, nullptr, 16);
	;
	VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1) == 1;
}

void Arm::rotate_clockwise() // 顺时针
{
	VCI_CAN_OBJ send[1];
	send[0].SendType = 0;
	send[0].RemoteFlag = 0;
	send[0].ExternFlag = 0;
	send[0].DataLen = 1;
	send[0].ID = 0X200 + 7;
	send[0].Data[0] = 0X00;
	VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1) == 1;
	gripperStatus[1] = '*';
	while (gripperStatus[1] == '*')
	{
		// std::cout << "gripperStatus:" << gripperStatus << std::endl;
	}
	if (logger)
		std::cout << "back gripperStatus:" << gripperStatus << std::endl;
	gripperStatus[6] = '1';
	std::string hexString = binaryStringToHexString(gripperStatus);
	if (logger)
		std::cout << "hexString:" << hexString << std::endl;

	send[0].SendType = 0;
	send[0].RemoteFlag = 0;
	send[0].ExternFlag = 0;
	send[0].DataLen = 1;
	send[0].ID = 0X100 + 7;
	send[0].Data[0] = std::stoul(hexString, nullptr, 16);
	VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1) == 1;
}

void Arm::rotate_anticlockwise() // 逆时针
{
	VCI_CAN_OBJ send[1];
	send[0].SendType = 0;
	send[0].RemoteFlag = 0;
	send[0].ExternFlag = 0;
	send[0].DataLen = 1;
	send[0].ID = 0X200 + 7;
	send[0].Data[0] = 0X00;
	VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1) == 1;
	gripperStatus[1] = '*';
	while (gripperStatus[1] == '*')
	{
		// std::cout << "gripperStatus:" << gripperStatus << std::endl;
	}
	if (logger)
		std::cout << "back gripperStatus:" << gripperStatus << std::endl;
	gripperStatus[6] = '0';
	std::string hexString = binaryStringToHexString(gripperStatus);
	if (logger)
		std::cout << "hexString:" << hexString << std::endl;

	send[0].SendType = 0;
	send[0].RemoteFlag = 0;
	send[0].ExternFlag = 0;
	send[0].DataLen = 1;
	send[0].ID = 0X100 + 7;
	send[0].Data[0] = std::stoul(hexString, nullptr, 16);
	;
	VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1) == 1;
}

void Arm::extend()
{
	VCI_CAN_OBJ send[1];
	send[0].SendType = 0;
	send[0].RemoteFlag = 0;
	send[0].ExternFlag = 0;
	send[0].DataLen = 1;
	send[0].ID = 0X200 + 7;
	send[0].Data[0] = 0X00;
	VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1) == 1;
	gripperStatus[1] = '*';
	while (gripperStatus[1] == '*')
	{
		// std::cout << "gripperStatus:" << gripperStatus << std::endl;
	}
	if (logger)
		std::cout << "back gripperStatus:" << gripperStatus << std::endl;
	gripperStatus[5] = '1';
	std::string hexString = binaryStringToHexString(gripperStatus);
	if (logger)
		std::cout << "hexString:" << hexString << std::endl;

	send[0].SendType = 0;
	send[0].RemoteFlag = 0;
	send[0].ExternFlag = 0;
	send[0].DataLen = 1;
	send[0].ID = 0X100 + 7;
	send[0].Data[0] = std::stoul(hexString, nullptr, 16);
	;
	VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1) == 1;
}

void Arm::retract()
{
	VCI_CAN_OBJ send[1];
	send[0].SendType = 0;
	send[0].RemoteFlag = 0;
	send[0].ExternFlag = 0;
	send[0].DataLen = 1;
	send[0].ID = 0X200 + 7;
	send[0].Data[0] = 0X00;
	VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1) == 1;
	gripperStatus[1] = '*';
	while (gripperStatus[1] == '*')
	{
		// std::cout << "gripperStatus:" << gripperStatus << std::endl;
	}
	if (logger)
		std::cout << "back gripperStatus:" << gripperStatus << std::endl;
	gripperStatus[5] = '0';
	std::string hexString = binaryStringToHexString(gripperStatus);
	if (logger)
		std::cout << "hexString:" << hexString << std::endl;

	send[0].SendType = 0;
	send[0].RemoteFlag = 0;
	send[0].ExternFlag = 0;
	send[0].DataLen = 1;
	send[0].ID = 0X100 + 7;
	send[0].Data[0] = std::stoul(hexString, nullptr, 16);
	;
	VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1) == 1;
}

bool Arm::isOpen()
{
	VCI_CAN_OBJ send[1];
	send[0].SendType = 0;
	send[0].RemoteFlag = 0;
	send[0].ExternFlag = 0;
	send[0].DataLen = 1;
	send[0].ID = 0X200 + 7;
	send[0].Data[0] = 0X00;
	VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1) == 1;
	gripperStatus[1] = '*';
	while (gripperStatus[1] == '*')
	{
		// std::cout << "gripperStatus:" << gripperStatus << std::endl;
	}
	if (logger)
	{
		std::cout << "back gripperStatus:" << gripperStatus << std::endl;
	}
	if (gripperStatus[7] == '1')
	{
		return true;
	}
	return false;
}

bool Arm::isClockwise()
{
	VCI_CAN_OBJ send[1];
	send[0].SendType = 0;
	send[0].RemoteFlag = 0;
	send[0].ExternFlag = 0;
	send[0].DataLen = 1;
	send[0].ID = 0X200 + 7;
	send[0].Data[0] = 0X00;
	VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1) == 1;
	gripperStatus[1] = '*';
	while (gripperStatus[1] == '*')
	{
		// std::cout << "gripperStatus:" << gripperStatus << std::endl;
	}
	if (logger)
	{
		std::cout << "back gripperStatus:" << gripperStatus << std::endl;
	}
	if (gripperStatus[6] == '1')
	{
		return true;
	}
	return false;
}

bool Arm::isextend()
{
	VCI_CAN_OBJ send[1];
	send[0].SendType = 0;
	send[0].RemoteFlag = 0;
	send[0].ExternFlag = 0;
	send[0].DataLen = 1;
	send[0].ID = 0X200 + 7;
	send[0].Data[0] = 0X00;
	VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1) == 1;
	gripperStatus[1] = '*';
	while (gripperStatus[1] == '*')
	{
		// std::cout << "gripperStatus:" << gripperStatus << std::endl;
	}
	if (logger)
	{
		std::cout << "back gripperStatus:" << gripperStatus << std::endl;
	}
	if (gripperStatus[5] == '1')
	{
		return true;
	}
	return false;
}