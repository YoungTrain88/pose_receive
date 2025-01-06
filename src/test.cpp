

#include <iostream>
#include <thread>
#include <vector>
#include <mutex>
#include <condition_variable>

#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>
#include "controlcan.h"
#include <iostream>
#include <ctime>
#include <cstdlib>
#include "unistd.h"

VCI_BOARD_INFO pInfo; // 用来获取设备信息。
int count = 0;		  // 数据列表中，用来存储列表序号。
VCI_BOARD_INFO pInfo1[50];
int num = 0;
long newPosition;  // 1轴目标位置，单位度
long newPosition2; // 2轴目标位置，单位mm
long newPosition3; // 3轴目标位置，单位mm
long newPosition4; // 4度轴目标位置，单位度
long Pos;		   // 1轴速度，单位rpm
long Pos2;		   // 2轴速度，单位rpm
long Pos3;		   // 3轴速度，单位rpm
long Pos4;		   // 4轴速度，单位rpm

void *receive_func(void *param) // 接收线程。
{
	int reclen = 0;
	VCI_CAN_OBJ rec[3000]; // 接收缓存，设为3000为佳。
	int i, j;

	int *run = (int *)param; // 线程启动，退出控制。
	int ind = 0;

	while ((*run) & 0x0f)
	{
		if ((reclen = VCI_Receive(VCI_USBCAN2, 0, ind, rec, 3000, 100)) > 0) // 调用接收函数，如果有数据，进行数据处理显示。
		{
			for (j = 0; j < reclen; j++)
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
					printf(" Remote ");				  // 帧类型：远程帧
				printf("DLC:0x%02X", rec[j].DataLen); // 帧长度
				printf(" data:0x");					  // 数据
				for (i = 0; i < rec[j].DataLen; i++)
				{
					printf(" %02X", rec[j].Data[i]);
				}
				printf(" TimeStamp:0x%08X", rec[j].TimeStamp); // 时间标识。
				printf("\n");
			}
		}
		ind = !ind; // 变换通道号，以便下次读取另一通道，交替读取。
	}
	printf("run thread exit\n"); // 退出接收线程
	pthread_exit(0);
}
std::mutex mtx;				// 互斥锁，用于保护共享数据
std::condition_variable cv; // 条件变量，用于线程间的同步
int currentTargetPosition;
bool newPositionReceived = false;
VCI_CAN_OBJ send[1];

// // 控制移动的函数   这是电机1
void moveArmToPosition()
{
	while (true)
	{
		std::unique_lock<std::mutex> lock(mtx); // 使用unique_lock来锁定互斥量
		cv.wait(lock, []
				{ return newPositionReceived; }); // 等待新位置的通知

		// //         // 当收到新位置时，执行移动操作
		std::cout << "Moving arm to new position: " << currentTargetPosition << std::endl;
		// //         // 这里添加实际控制电机的代码
		newPositionReceived = false; // 重置标志位，等待下一个位置
	};
}
// 接收新的目标位置的函数
void receivePosition()

{
	while (true)
	{

		// 1号电机位置
		newPosition;
		std::cout << "Enter new target position: ";
		std::cin >> newPosition;
		int newvel;
		std::cout << "Enter new target vel: ";
		std::cin >> newvel; // 1轴速度
		int64_t send1 = (int64_t)newvel * 512 * 65536 / 1875;
		long Pos = newPosition * (7864320 / 360);

		newPosition2;
		std::cout << "Enter new target position2: ";
		std::cin >> newPosition2;
		int newvel2;
		std::cout << "Enter new target vel2: ";
		std::cin >> newvel2; // 2轴速度
		int64_t send2 = (int64_t)newvel2 * 512 * 65536 / 1875;
		long Pos2 = newPosition2 * (262144 / 120);

		newPosition3;
		std::cout << "Enter new target position3: ";
		std::cin >> newPosition3;
		int newvel3;
		std::cout << "Enter new target vel3: ";
		std::cin >> newvel3; // 3轴速度
		int64_t send3 = (int64_t)newvel3 * 512 * 65536 / 1875;
		long Pos3 = newPosition3 * (262144 / 120);

		newPosition4;
		std::cout << "Enter new target position4: ";
		std::cin >> newPosition4;
		int newvel4;
		std::cout << "Enter new target vel4: ";
		std::cin >> newvel4; // 4轴速度
		int64_t send4 = (int64_t)newvel4 * 2184;
		long Pos4 = newPosition4 * (131072 / 360);

		int number;
		std::cout << "Enter motor choice: ";
		std::cin >> number; // 几号电机

		if (number == 1)
		{
			std::cout << "当前进入一号电机！" << std::endl;

			send[0].Data[3] = (send1 >> 24) & 0xFF;
			send[0].Data[2] = (send1 >> 16) & 0xFF;
			send[0].Data[1] = (send1 >> 8) & 0xFF;
			send[0].Data[0] = send1 & 0xFF;
			send[0].ID = 0X201;
			send[0].Data[7] = (Pos >> 24) & 0xFF;
			send[0].Data[6] = (Pos >> 16) & 0xFF;
			send[0].Data[5] = (Pos >> 8) & 0xFF;
			send[0].Data[4] = Pos & 0xFF;
			VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1);
		}
		if (number == 2)
		{
			send[0].Data[3] = (send2 >> 24) & 0xFF;
			send[0].Data[2] = (send2 >> 16) & 0xFF;
			send[0].Data[1] = (send2 >> 8) & 0xFF;
			send[0].Data[0] = send2 & 0xFF;
			send[0].ID = 0X202;
			send[0].Data[7] = (Pos2 >> 24) & 0xFF;
			send[0].Data[6] = (Pos2 >> 16) & 0xFF;
			send[0].Data[5] = (Pos2 >> 8) & 0xFF;
			send[0].Data[4] = Pos2 & 0xFF;
			VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1);
			send[0].Data[3] = (send3 >> 24) & 0xFF;
			send[0].Data[2] = (send3 >> 16) & 0xFF;
			send[0].Data[1] = (send3 >> 8) & 0xFF;
			send[0].Data[0] = send3 & 0xFF;
			send[0].ID = 0X203;
			send[0].Data[7] = (Pos3 >> 24) & 0xFF;
			send[0].Data[6] = (Pos3 >> 16) & 0xFF;
			send[0].Data[5] = (Pos3 >> 8) & 0xFF;
			send[0].Data[4] = Pos3 & 0xFF;
			VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1);
		}
		// 3号电机位置
		if (number == 3)
		{

			send[0].Data[3] = (send3 >> 24) & 0xFF;
			send[0].Data[2] = (send3 >> 16) & 0xFF;
			send[0].Data[1] = (send3 >> 8) & 0xFF;
			send[0].Data[0] = send3 & 0xFF;
			send[0].ID = 0X203;
			send[0].Data[7] = (Pos3 >> 24) & 0xFF;
			send[0].Data[6] = (Pos3 >> 16) & 0xFF;
			send[0].Data[5] = (Pos3 >> 8) & 0xFF;
			send[0].Data[4] = Pos3 & 0xFF;
			VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1);
		}
		// 4号电机位置
		if (number == 4)
		{
			send[0].ID = 0X604;
			send[0].Data[0] = 0X23;
			send[0].Data[1] = 0X81;
			send[0].Data[2] = 0X60;
			send[0].Data[3] = 0X00;
			send[0].Data[4] = send4 & 0xFF;
			send[0].Data[5] = (send4 >> 8) & 0xFF;
			send[0].Data[6] = (send4 >> 16) & 0xFF;
			send[0].Data[7] = (send4 >> 24) & 0xFF;
			VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1);
			send[0].ID = 0X604;
			send[0].Data[0] = 0X23;
			send[0].Data[1] = 0X7A;
			send[0].Data[2] = 0X60;
			send[0].Data[3] = 0X00;
			send[0].Data[4] = Pos4 & 0xFF;
			send[0].Data[5] = (Pos4 >> 8) & 0xFF;
			send[0].Data[6] = (Pos4 >> 16) & 0xFF;
			send[0].Data[7] = (Pos4 >> 24) & 0xFF;
			VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1);

			send[0].ID = 0X604;
			send[0].Data[0] = 0X2B;
			send[0].Data[1] = 0X40;
			send[0].Data[2] = 0X60;
			send[0].Data[3] = 0X00;
			send[0].Data[4] = 0X06;
			send[0].Data[5] = 0X00;
			send[0].Data[6] = 0X00;
			send[0].Data[7] = 0X00;
			VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1);
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
			send[0].Data[4] = 0X1F;
			send[0].Data[5] = 0X00;
			send[0].Data[6] = 0X00;
			send[0].Data[7] = 0X00;
			VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1);
		}
		std::lock_guard<std::mutex> lock(mtx); // 使用lock_guard来自动锁定和解锁互斥量

		// currentTargetPosition = newPosition;  // 更新目标位置
		newPositionReceived = true; // 设置新位置接收标志
		cv.notify_one();			// 通知等待的线程
	}
}

main()
{
	std::vector<int> ids = {1, 2, 3, 4};

	printf(">程序以运行 !\r\n"); // 指示程序已运行
	num = VCI_FindUsbDevice2(pInfo1);
	printf(">>USBCAN DEVICE NUM:");
	printf("%d", num);
	printf(" PCS");
	printf("\n");
	for (int i = 0; i < num; i++)
	{
		printf("Device:");
		printf("%d", i);
		printf("\n");
		printf(">>Get VCI_ReadBoardInfo success!\n");
		printf(">>Serial_Num:%c", pInfo1[i].str_Serial_Num[0]);
		printf("%c", pInfo1[i].str_Serial_Num[1]);
		printf("%c", pInfo1[i].str_Serial_Num[2]);
		printf("%c", pInfo1[i].str_Serial_Num[3]);
		printf("%c", pInfo1[i].str_Serial_Num[4]);
		printf("%c", pInfo1[i].str_Serial_Num[5]);
		printf("%c", pInfo1[i].str_Serial_Num[6]);
		printf("%c", pInfo1[i].str_Serial_Num[7]);
		printf("%c", pInfo1[i].str_Serial_Num[8]);
		printf("%c", pInfo1[i].str_Serial_Num[9]);
		printf("%c", pInfo1[i].str_Serial_Num[10]);
		printf("%c", pInfo1[i].str_Serial_Num[11]);
		printf("%c", pInfo1[i].str_Serial_Num[12]);
		printf("%c", pInfo1[i].str_Serial_Num[13]);
		printf("%c", pInfo1[i].str_Serial_Num[14]);
		printf("%c", pInfo1[i].str_Serial_Num[15]);
		printf("%c", pInfo1[i].str_Serial_Num[16]);
		printf("%c", pInfo1[i].str_Serial_Num[17]);
		printf("%c", pInfo1[i].str_Serial_Num[18]);
		printf("%c", pInfo1[i].str_Serial_Num[19]);
		printf("\n");
		printf(">>hw_Type:%c", pInfo1[i].str_hw_Type[0]);
		printf("%c", pInfo1[i].str_hw_Type[1]);
		printf("%c", pInfo1[i].str_hw_Type[2]);
		printf("%c", pInfo1[i].str_hw_Type[3]);
		printf("%c", pInfo1[i].str_hw_Type[4]);
		printf("%c", pInfo1[i].str_hw_Type[5]);
		printf("%c", pInfo1[i].str_hw_Type[6]);
		printf("%c", pInfo1[i].str_hw_Type[7]);
		printf("%c", pInfo1[i].str_hw_Type[8]);
		printf("%c", pInfo1[i].str_hw_Type[9]);
		printf("\n");
		printf(">>Firmware Version:V");
		printf("%x", (pInfo1[i].fw_Version & 0xF00) >> 8);
		printf(".");
		printf("%x", (pInfo1[i].fw_Version & 0xF0) >> 4);
		printf("%x", pInfo1[i].fw_Version & 0xF);
		printf("\n");
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
	if (VCI_ReadBoardInfo(VCI_USBCAN2, 0, &pInfo) == 1) // 读取设备序列号、版本等信息。
	{
		printf(">>Get VCI_ReadBoardInfo success!\n");
		printf(">>Serial_Num:%c", pInfo.str_Serial_Num[0]);
		printf("%c", pInfo.str_Serial_Num[1]);
		printf("%c", pInfo.str_Serial_Num[2]);
		printf("%c", pInfo.str_Serial_Num[3]);
		printf("%c", pInfo.str_Serial_Num[4]);
		printf("%c", pInfo.str_Serial_Num[5]);
		printf("%c", pInfo.str_Serial_Num[6]);
		printf("%c", pInfo.str_Serial_Num[7]);
		printf("%c", pInfo.str_Serial_Num[8]);
		printf("%c", pInfo.str_Serial_Num[9]);
		printf("%c", pInfo.str_Serial_Num[10]);
		printf("%c", pInfo.str_Serial_Num[11]);
		printf("%c", pInfo.str_Serial_Num[12]);
		printf("%c", pInfo.str_Serial_Num[13]);
		printf("%c", pInfo.str_Serial_Num[14]);
		printf("%c", pInfo.str_Serial_Num[15]);
		printf("%c", pInfo.str_Serial_Num[16]);
		printf("%c", pInfo.str_Serial_Num[17]);
		printf("%c", pInfo.str_Serial_Num[18]);
		printf("%c", pInfo.str_Serial_Num[19]);
		printf("\n");
		printf(">>hw_Type:%c", pInfo.str_hw_Type[0]);
		printf("%c", pInfo.str_hw_Type[1]);
		printf("%c", pInfo.str_hw_Type[2]);
		printf("%c", pInfo.str_hw_Type[3]);
		printf("%c", pInfo.str_hw_Type[4]);
		printf("%c", pInfo.str_hw_Type[5]);
		printf("%c", pInfo.str_hw_Type[6]);
		printf("%c", pInfo.str_hw_Type[7]);
		printf("%c", pInfo.str_hw_Type[8]);
		printf("%c", pInfo.str_hw_Type[9]);
		printf("\n");
		printf(">>Firmware Version:V");
		printf("%x", (pInfo.fw_Version & 0xF00) >> 8);
		printf(".");
		printf("%x", (pInfo.fw_Version & 0xF0) >> 4);
		printf("%x", pInfo.fw_Version & 0xF);
		printf("\n");
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
	config.Timing0 = 0x00; /*波特率1000 Kbps  0x00  0x14*/
	config.Timing1 = 0x14;
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
	// 需要发送的帧，结构体设置
	send[0].ID = 0;
	send[0].SendType = 0;
	send[0].RemoteFlag = 0;
	send[0].ExternFlag = 0;
	send[0].DataLen = 8;

	// 再上伺服
	for (size_t i = 0; i < 3; i++)
	{
		send[0].ID = 0X600 + ids[i];
		send[0].Data[0] = 0X2B;
		send[0].Data[1] = 0X40;
		send[0].Data[2] = 0X60;
		send[0].Data[3] = 0X00;
		send[0].Data[4] = 0X06;
		send[0].Data[5] = 0X00;
		send[0].Data[6] = 0X00;
		send[0].Data[7] = 0X00;
		VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1) == 1;
	}
	// 上伺服
	for (size_t i = 0; i < 3; i++)
	{
		send[0].ID = 0X600 + ids[i];
		send[0].Data[0] = 0X2B;
		send[0].Data[1] = 0X40;
		send[0].Data[2] = 0X60;
		send[0].Data[3] = 0X00;
		send[0].Data[4] = 0X0F;
		send[0].Data[5] = 0X00;
		send[0].Data[6] = 0X00;
		send[0].Data[7] = 0X00;
		VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1) == 1;
	}
	// 走绝对位置的模式
	for (size_t i = 0; i < 3; i++)
	{
		send[0].ID = 0X600 + ids[i];
		send[0].Data[0] = 0X2B;
		send[0].Data[1] = 0X40;
		send[0].Data[2] = 0X60;
		send[0].Data[3] = 0X00;
		send[0].Data[4] = 0X3F;
		send[0].Data[5] = 0X10;
		send[0].Data[6] = 0X00;
		send[0].Data[7] = 0X00;
		VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1) == 1;
	}
	// 控制模式：位置模式
	for (size_t i = 0; i < 3; i++)
	{
		send[0].ID = 0X600 + ids[i];
		send[0].Data[0] = 0X2F;
		send[0].Data[1] = 0X60;
		send[0].Data[2] = 0X60;
		send[0].Data[3] = 0X00;
		send[0].Data[4] = 0X01;
		send[0].Data[5] = 0X10;
		send[0].Data[6] = 0X00;
		send[0].Data[7] = 0X00;
		VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1) == 1;
	}

	// 开启节点
	for (size_t i = 0; i < 3; i++)
	{
		send[0].ID = 0X00;
		send[0].Data[0] = 0X01;
		send[0].Data[1] = 0X00 + i;
		VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1) == 1;
	}

	int i = 0;
	int m_run0 = 1;
	pthread_t threadid; // 创建线程
	int ret;
	ret = pthread_create(&threadid, NULL, receive_func, &m_run0); // 应该是线程池个数
	VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1);

	std::thread movingThread(moveArmToPosition); // 创建控制机械臂移动的线程
	receivePosition();
	std::thread receivingThread(receivePosition); // 创建接收新位置的线程
	movingThread.join();						  // 等待移动线程结束
	receivingThread.join();						  // 等待接收线程结束

	usleep(10000000);				 // 延时单位us，这里设置 10 000 000=10s    10s后关闭接收线程，并退出主程序。
	m_run0 = 0;						 // 线程关闭指令。
	pthread_join(threadid, NULL);	 // 等待线程关闭。
	usleep(100000);					 // 延时100ms。
	VCI_ResetCAN(VCI_USBCAN2, 0, 0); // 复位CAN1通道。
	usleep(100000);					 // 延时100ms。
	VCI_ResetCAN(VCI_USBCAN2, 0, 1); // 复位CAN2通道。
	usleep(100000);					 // 延时100ms。
	VCI_CloseDevice(VCI_USBCAN2, 0); // 关闭设备。
	// 除收发函数外，其它的函数调用前后，最好加个毫秒级的延时，即不影响程序的运行，又可以让USBCAN设备有充分的时间处理指令。
	// goto ext;
}
