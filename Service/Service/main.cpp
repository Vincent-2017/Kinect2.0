#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include "winsock2.h"
#include<cstdlib>
#pragma comment(lib,"ws2_32.lib")//引用库文件
using namespace std;


char recvBuf[100];
SOCKET sockConn;
/**
* 在一个新的线程里面接收数据
*/
DWORD WINAPI Fun(LPVOID lpParamter)
{
	while (true){
		memset(recvBuf, 0, sizeof(recvBuf));
		//      //接收数据
		recv(sockConn, recvBuf, sizeof(recvBuf), 0);
		printf("%s\n", recvBuf);
	}
	closesocket(sockConn);
}

int main()
{
	WSADATA wsaData;
	int port = 8080;//端口号
	// 加载套接字库
	if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0)
	{
		printf("初始化失败");
		return 0;
	}
	// 创建用于监听的套接字,即服务端的套接字
	SOCKET sockSrv = socket(AF_INET, SOCK_STREAM, 0);
	// 配置服务器地址
	SOCKADDR_IN addrSrv;
	addrSrv.sin_family = AF_INET;
	addrSrv.sin_port = htons(port); //1024以上的端口号
	addrSrv.sin_addr.S_un.S_addr = inet_addr("192.168.137.1");
	// 绑定
	int retVal = bind(sockSrv, (LPSOCKADDR)&addrSrv, sizeof(SOCKADDR_IN));
	if (retVal == SOCKET_ERROR){
		printf("连接失败:%d\n", WSAGetLastError());
		return 0;
	}
	// 监听
	if (listen(sockSrv, 10) == SOCKET_ERROR){
		printf("监听失败:%d", WSAGetLastError());
		return 0;
	}

	SOCKADDR_IN addrClient;
	int len = sizeof(SOCKADDR);

	while (1)
	{
		//等待客户请求到来
		sockConn = accept(sockSrv, (SOCKADDR *)&addrClient, &len);
		if (sockConn == SOCKET_ERROR){
			printf("等待请求失败:%d", WSAGetLastError());
			break;
		}
		printf("客户端的IP是:[%s]\n", inet_ntoa(addrClient.sin_addr));

		//发送数据
		char sendbuf[] = "服务端IP:192.168.137.1 Port:8080";

		int iSend = send(sockConn, sendbuf, sizeof(sendbuf), 0);
		if (iSend == SOCKET_ERROR){
			printf("发送失败");
			break;
		}

		HANDLE hThread = CreateThread(NULL, 0, Fun, NULL, 0, NULL);
		CloseHandle(hThread);
	}

	closesocket(sockSrv);
	WSACleanup();
	system("pause");
	return 0;
}