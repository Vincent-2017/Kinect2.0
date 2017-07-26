#include<iostream>
#include<opencv2\opencv.hpp>
#include "kinect.h"  
#include<string>
using namespace cv;

void DrawMap(Mat img, float angle, float confidence);
int main()
{

	IKinectSensor* pKinectSensor; //申请一个Sensor指针  
	HRESULT hr = GetDefaultKinectSensor(&pKinectSensor); // 获取一个默认的Sensor  

	BOOLEAN bIsOpen = 0;
	pKinectSensor->get_IsOpen(&bIsOpen); // 查看下是否已经打开  
	printf("bIsOpen： %d\n", bIsOpen);

	if (!bIsOpen) // 没打开，则尝试打开  
	{
		hr = pKinectSensor->Open();
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
	WCHAR bbuid[256] = { 0 };
	pKinectSensor->get_UniqueKinectId(256, bbuid); // 获取唯一ID  
	printf("UID: %s\n", bbuid);

	// 音频数据获取  
	IAudioSource* audios = nullptr;
	UINT nAudioCount = 0;
	hr = pKinectSensor->get_AudioSource(&audios);
	IAudioBeam* audiobm = nullptr;
	IAudioBeamList* audiobml = nullptr;
	audios->get_AudioBeams(&audiobml);
	audiobml->OpenAudioBeam(0, &audiobm); // 目前只支持第一个  

	float fAngle = 0.0f;
	float fAngleConfidence = 0.0f;

	while (waitKey(30) != 27)
	{
		Mat img(200, 400, CV_8UC3, Scalar(255, 255, 255));
		//	circle(img, Point(200, 0), 100, Scalar(0, 0, 0));
		fAngle = 0.0f;
		fAngleConfidence = 0.0f;
		audiobm->get_BeamAngle(&fAngle); // 获取音频的角度
		audiobm->get_BeamAngleConfidence(&fAngleConfidence); // 获取音频的可信度（0 - 1）  
		DrawMap(img, fAngle, fAngleConfidence);
		imshow("", img);
	}


	pKinectSensor->Close();

	return 0;
}

void DrawMap(Mat img, float angle, float confidence)
{
	circle(img, Point(200, 0), 100, Scalar(0, 0, 0));
	Point point;
	point.x = 100 * sin(angle) + 200;
	point.y = 100 * cos(angle);
	if (confidence > 0.5)
		line(img, Point(200, 0), point, Scalar(0, 0, 255));
	std::string str = "angle:" + std::to_string((angle / 3.1415926f)*180.0f) + " confidence:" + std::to_string(confidence);
	putText(img, str, Point(20, 180), CV_FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 0, 0), 1, 4);


}