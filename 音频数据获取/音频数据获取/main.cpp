#include "iostream"  
#include "kinect.h"  
#define _USE_MATH_DEFINES  
#include <math.h>  

int main()
{
	printf("Hello, Wellcome to kinect world!\n");
	IKinectSensor* pKinectSensor; //申请一个Sensor指针  
	HRESULT hr = GetDefaultKinectSensor(&pKinectSensor); // 获取一个默认的Sensor  
	if (FAILED(hr))
	{
		printf("No Kinect connect to your pc!\n");
		goto endstop;
	}
	BOOLEAN bIsOpen = 0;
	pKinectSensor->get_IsOpen(&bIsOpen); // 查看下是否已经打开  
	printf("bIsOpen： %d\n", bIsOpen);

	if (!bIsOpen) // 没打开，则尝试打开  
	{
		hr = pKinectSensor->Open();
		if (FAILED(hr))
		{
			printf("Kinect Open Failed!\n");
			goto endstop;
		}
		printf("Kinect opened! But it need sometime to work!\n");
		// 这里一定要多等会，否则下面的判断都是错误的  
		printf("Wait For 3000 ms...\n");
		Sleep(3000);
	}
	bIsOpen = 0;
	pKinectSensor->get_IsOpen(&bIsOpen); // 是否已经打开  
	printf("bIsOpen： %d\n", bIsOpen);
	BOOLEAN bAvaliable = 0;
	pKinectSensor->get_IsAvailable(&bAvaliable); // 是否可用  
	printf("bAvaliable： %d\n", bAvaliable);

	DWORD dwCapability = 0;
	pKinectSensor->get_KinectCapabilities(&dwCapability); // 获取容量  
	printf("dwCapability： %d\n", dwCapability);
	TCHAR bbuid[256] = { 0 };
	pKinectSensor->get_UniqueKinectId(256, bbuid); // 获取唯一ID  
	printf("UID: %s\n", bbuid);

	// 音频数据获取  
	IAudioSource* pAudioSource = nullptr;
	// 音频帧读取器
	IAudioBeamFrameReader*  pAudioBeamFrameReader = nullptr;
	// 音频帧引用
	IAudioBeamFrameReference* pABFrameRef = nullptr;
	// 音频帧链表
	IAudioBeamFrameList* pAudioBeamFrameList = nullptr;
	// 音频帧
	IAudioBeamFrame* pAudioBeamFrame = nullptr;
	hr = pKinectSensor->get_AudioSource(&pAudioSource);
	if (FAILED(hr))
	{
		printf("Audio Source get failed!\n");
		goto endclose;
	}
	// 再获取音频帧读取器
	if (SUCCEEDED(hr)){
		hr = pAudioSource->OpenReader(&pAudioBeamFrameReader);
	}
	
endclose:
	pKinectSensor->Close();
endstop:
	system("pause");
	return 0;
}