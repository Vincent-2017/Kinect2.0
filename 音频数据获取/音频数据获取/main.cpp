#include "iostream"  
#include "kinect.h"  
#define _USE_MATH_DEFINES  
#include <math.h>  

int main()
{
	printf("Hello, Wellcome to kinect world!\n");
	IKinectSensor* bb; //申请一个Sensor指针  
	HRESULT hr = GetDefaultKinectSensor(&bb); // 获取一个默认的Sensor  
	if (FAILED(hr))
	{
		printf("No Kinect connect to your pc!\n");
		goto endstop;
	}
	BOOLEAN bIsOpen = 0;
	bb->get_IsOpen(&bIsOpen); // 查看下是否已经打开  
	printf("bIsOpen： %d\n", bIsOpen);

	if (!bIsOpen) // 没打开，则尝试打开  
	{
		hr = bb->Open();
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
	bb->get_IsOpen(&bIsOpen); // 是否已经打开  
	printf("bIsOpen： %d\n", bIsOpen);
	BOOLEAN bAvaliable = 0;
	bb->get_IsAvailable(&bAvaliable); // 是否可用  
	printf("bAvaliable： %d\n", bAvaliable);

	DWORD dwCapability = 0;
	bb->get_KinectCapabilities(&dwCapability); // 获取容量  
	printf("dwCapability： %d\n", dwCapability);
	TCHAR bbuid[256] = { 0 };
	bb->get_UniqueKinectId(256, bbuid); // 获取唯一ID  
	printf("UID: %s\n", bbuid);

	// 音频数据获取  
	IAudioSource* audios = nullptr;
	UINT nAudioCount = 0;
	hr = bb->get_AudioSource(&audios);
	if (FAILED(hr))
	{
		printf("Audio Source get failed!\n");
		goto endclose;
	}
	IAudioBeam* audiobm = nullptr;
	IAudioBeamList* audiobml = nullptr;
	audios->get_AudioBeams(&audiobml);
	audiobml->OpenAudioBeam(0, &audiobm); // 目前只支持第一个  
	IStream* stm = nullptr;
	audiobm->OpenInputStream(&stm);
	audios->Release();
	audiobm->Release();

	float fAngle = 0.0f;
	float fAngleConfidence = 0.0f;
	ULONG lRead = 0;
	const ULONG lBufferSize = 3200;
	float* fDataArr = new float[lBufferSize];
	while (true)
	{
		fAngle = 0.0f;
		fAngleConfidence = 0.0f;
		audiobm->get_BeamAngle(&fAngle); // 获取音频的角度，[ -0.872665f, 0.8726665f ]  
		audiobm->get_BeamAngleConfidence(&fAngleConfidence); // 获取音频的可信度（0 - 1）  
		if (fAngleConfidence > 0.5f)
			printf("Angle: %3.2f (%1.2f)\n", (fAngle)*180.0f / static_cast<float>(M_PI), fAngleConfidence);
		// audio  data  
		lRead = 0;
		memset(fDataArr, 0, lBufferSize);
		stm->Read(fDataArr, lBufferSize, &lRead);
		if (lRead > 0)
		{
			printf("Audio Buffer: %d\n", lRead);
		}
		Sleep(200);
	}

endclose:
	bb->Close();
endstop:
	system("pause");
	return 0;
}