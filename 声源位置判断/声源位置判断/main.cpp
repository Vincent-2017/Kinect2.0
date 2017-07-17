#include "kinect.h"
#include "iostream"

int main()
{
	printf("Hello, Wellcome to kinect world!\n");
	IKinectSensor* pKinect; //申请一个Sensor指针
	HRESULT hr = GetDefaultKinectSensor(&pKinect); // 获取一个默认的Sensor
	if (FAILED(hr))
	{
		printf("No Kinect connect to your pc!\n");
		goto endstop;
	}
	BOOLEAN bIsOpen = 0;
	pKinect->get_IsOpen(&bIsOpen); // 查看下是否已经打开
	printf("bIsOpen： %d\n", bIsOpen);

	if (!bIsOpen) // 没打开，则尝试打开
	{
		hr = pKinect->Open();
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
	pKinect->get_IsOpen(&bIsOpen); // 是否已经打开
	printf("bIsOpen： %d\n", bIsOpen);
	BOOLEAN bAvaliable = 0;
	pKinect->get_IsAvailable(&bAvaliable); // 是否可用
	printf("bAvaliable： %d\n", bAvaliable);

	DWORD dwCapability = 0;
	pKinect->get_KinectCapabilities(&dwCapability); // 获取容量
	printf("dwCapability： %d\n", dwCapability);
	TCHAR bbuid[256] = { 0 };
	pKinect->get_UniqueKinectId(256, bbuid); // 获取唯一ID
	printf("UID: %s\n", bbuid);

	// 音频数据获取
	IAudioSource* audios = nullptr;
	UINT nAudioCount = 0;
	hr = pKinect->get_AudioSource(&audios);
	if (FAILED(hr))
	{
		printf("Audio Source get failed!\n");
		goto endclose;
	}
	IAudioBeam* audiobm = nullptr;
	IAudioBeamList* audiobml = nullptr;
	audios->get_AudioBeams(&audiobml);
	audiobml->OpenAudioBeam(0, &audiobm); // 目前只支持第一个

	float fAngle = 0.0f;
	float fAngleConfidence = 0.0f;
	while (true)
	{
		fAngle = 0.0f;
		fAngleConfidence = 0.0f;
		audiobm->get_BeamAngle(&fAngle); // 获取音频的角度，[ -0.872665f, 0.8726665f ]
		audiobm->get_BeamAngleConfidence(&fAngleConfidence); // 获取音频的可信度（0 - 1）
		printf("Angle: %3.2f (%1.2f)\n", (fAngle / 3.1415926f)*180.0f, fAngleConfidence);
		Sleep(200);
	}

endclose:
	pKinect->Close();
endstop:
	system("pause");
	return 0;
}
