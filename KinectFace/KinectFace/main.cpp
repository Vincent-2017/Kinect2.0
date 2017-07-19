#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"
#include <iostream>
#include <Kinect.h>  
#include<Kinect.Face.h>
#pragma comment ( lib, "kinect20.lib" )  
#pragma comment ( lib, "Kinect20.face.lib" )  

using namespace cv;
using namespace std;

// 安全释放对象
template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL)
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}


int main()
{
	// 打开kinect
	IKinectSensor*kinect;
	GetDefaultKinectSensor(&kinect);
	kinect->Open();
	HRESULT hResult;

	// 彩色数据
	IColorFrameSource*colorsource;
	kinect->get_ColorFrameSource(&colorsource);
	IColorFrameReader*colorreader;
	colorsource->OpenReader(&colorreader);
	// 骨骼数据
	IBodyFrameSource*bodysource;
	kinect->get_BodyFrameSource(&bodysource);
	IBodyFrameReader*bodyreader;
	bodysource->OpenReader(&bodyreader);
	// 坐标映射
	ICoordinateMapper* coordinatemapper;
	kinect->get_CoordinateMapper(&coordinatemapper);

	// 面部数据源
	IFaceFrameSource* facesource[BODY_COUNT];
	IFaceFrameReader* facereader[BODY_COUNT];

	// 面部特征
	DWORD features = FaceFrameFeatures::FaceFrameFeatures_BoundingBoxInColorSpace
		| FaceFrameFeatures::FaceFrameFeatures_PointsInColorSpace
		| FaceFrameFeatures::FaceFrameFeatures_RotationOrientation
		| FaceFrameFeatures::FaceFrameFeatures_Happy
		| FaceFrameFeatures::FaceFrameFeatures_RightEyeClosed
		| FaceFrameFeatures::FaceFrameFeatures_LeftEyeClosed
		| FaceFrameFeatures::FaceFrameFeatures_MouthOpen
		| FaceFrameFeatures::FaceFrameFeatures_MouthMoved
		| FaceFrameFeatures::FaceFrameFeatures_LookingAway
		| FaceFrameFeatures::FaceFrameFeatures_Glasses
		| FaceFrameFeatures::FaceFrameFeatures_FaceEngagement;
	
	// 获得面部数据源
	for (int i = 0; i < BODY_COUNT; i++)
	{
		hResult = CreateFaceFrameSource(kinect, 0, features, &facesource[i]);
		if (FAILED(hResult))
		{
			std::cerr << "Error : CreateFaceFrameSource" << std::endl;
			return -1;
		}
		facesource[i]->OpenReader(&facereader[i]);
	}

	// 主循环程序
	while (1)
	{
		Mat asd(1080, 1920, CV_8UC4);
		// 获得彩色帧并放入数组
		IColorFrame* colorframe = nullptr;
		hResult = colorreader->AcquireLatestFrame(&colorframe);
		if (colorframe == nullptr)
			continue;
		if (SUCCEEDED(hResult)) {
			colorframe->CopyConvertedFrameDataToArray(1920 * 1080 * 4, reinterpret_cast<BYTE*>(asd.data), ColorImageFormat::ColorImageFormat_Bgra);
		}
		// 释放彩色帧
		SafeRelease(colorframe);

		// 获得骨骼帧
		IBodyFrame* bodyframe = nullptr;
		hResult = bodyreader->AcquireLatestFrame(&bodyframe);
		if (SUCCEEDED(hResult))
		{
			IBody* body[BODY_COUNT] = { 0 };
			hResult = bodyframe->GetAndRefreshBodyData(BODY_COUNT, body);
			if (SUCCEEDED(hResult))
			{
#pragma omp parallel for //openMP并行程序设计
				for (int i = 0; i < BODY_COUNT; i++)
				{
					// 确认骨骼被追踪
					BOOLEAN tracked = false;
					hResult = body[i]->get_IsTracked(&tracked);
					if (SUCCEEDED(hResult) && tracked)
					{
						UINT64 trackingId = _UI64_MAX;
						hResult = body[i]->get_TrackingId(&trackingId);
						if (SUCCEEDED(hResult)) {
							// 将追踪的骨骼ID赋值给面源ID
							facesource[i]->put_TrackingId(trackingId);
							cout << "追踪的面部ID：" << trackingId << endl;
						}
					}
				}
			}
			for (int i = 0; i < BODY_COUNT; i++)
			{
				// 回收内存
				SafeRelease(body[i]);
			}
		}
		// 回收内存
		SafeRelease(bodyframe);
#pragma omp parallel for  //openMP并行程序设计
		for (int i = 0; i < BODY_COUNT; i++)
		{
			IFaceFrame*faceframe = nullptr;
			hResult = facereader[i]->AcquireLatestFrame(&faceframe);
			if (faceframe == nullptr)
				continue;
			if (SUCCEEDED(hResult) && faceframe != nullptr)
			{
				// 检查是否跟踪
				BOOLEAN tracked = false;
				hResult = faceframe->get_IsTrackingIdValid(&tracked);
				if (SUCCEEDED(hResult) && tracked)
				{
					// 获取面部帧结果
					IFaceFrameResult *faceresult = nullptr;
					hResult = faceframe->get_FaceFrameResult(&faceresult);
					if (SUCCEEDED(hResult))
					{
						// 获取彩色空间的面部特征点位置
						PointF facepoint[FacePointType_Count];
						hResult = faceresult->GetFacePointsInColorSpace(FacePointType_Count, facepoint);
						// 绘出面部特征点
						if (SUCCEEDED(hResult))
						{
							circle(asd, cv::Point(facepoint[0].X, facepoint[0].Y), 5, Scalar(0, 0, 255, 255), -1, CV_AA); // Eye (Left)
							circle(asd, cv::Point(facepoint[1].X, facepoint[1].Y), 5, Scalar(0, 0, 255, 255), -1, CV_AA); // Eye (Right)
							circle(asd, cv::Point(facepoint[2].X, facepoint[2].Y), 5, Scalar(0, 0, 255, 255), -1, CV_AA); // Nose
							circle(asd, cv::Point(facepoint[3].X, facepoint[3].Y), 5, Scalar(0, 0, 255, 255), -1, CV_AA); // Mouth (Left)
							circle(asd, cv::Point(facepoint[4].X, facepoint[4].Y), 5, Scalar(0, 0, 255, 255), -1, CV_AA); // Mouth (Right)
						}
						// 获取彩色空间的面部边框位置
						RectI box;
						hResult = faceresult->get_FaceBoundingBoxInColorSpace(&box);
						// 绘出面部边框
						if (SUCCEEDED(hResult))
						{
							cv::rectangle(asd, cv::Rect(box.Left, box.Top, box.Right - box.Left, box.Bottom - box.Top), Scalar(0, 0, 255, 255));
						}
					}
					// 回收内存
					SafeRelease(faceresult);
				}
			}
			// 回收内存
			SafeRelease(faceframe);
		}

		// 在图中显示最终结果
		Mat faceimg;
		cv::resize(asd, faceimg, cv::Size(), 0.5, 0.5);
		cv::imshow("Face", faceimg);
		// 按键 ESC 退出程序
		if (cv::waitKey(34) == VK_ESCAPE)
		{
			break;
		}
	}

	SafeRelease(colorsource);
	SafeRelease(bodysource);

	SafeRelease(colorreader);
	SafeRelease(bodyreader);

	SafeRelease(coordinatemapper);

	for (int i = 0; i < BODY_COUNT; i++)
	{
		SafeRelease(facesource[i]);
		SafeRelease(facereader[i]);
	}
	if (kinect) 
	{
		kinect->Close();
	}
	SafeRelease(kinect);
	cv::destroyAllWindows();

}