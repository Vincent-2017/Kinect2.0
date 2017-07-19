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

template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL)
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}

// 脸部位姿状态   四元数->俯仰、摇头、侧头
inline void ExtractFaceRotationInDegrees(const Vector4* pQuaternion, int* pPitch, int* pYaw, int* pRoll)
{
	double x = pQuaternion->x;
	double y = pQuaternion->y;
	double z = pQuaternion->z;
	double w = pQuaternion->w;

	// 弧度制换成角度制
	*pPitch = static_cast<int>(std::atan2(2 * (y * z + w * x), w * w - x * x - y * y + z * z) / 3.14159265358979 * 180.0f);
	*pYaw = static_cast<int>(std::asin(2 * (w * y - x * z)) / 3.14159265358979 * 180.0f);
	*pRoll = static_cast<int>(std::atan2(2 * (x * y + w * z), w * w + x * x - y * y - z * z) / 3.14159265358979 * 180.0f);
}

int main()
{

	IKinectSensor*kinect;
	GetDefaultKinectSensor(&kinect);
	kinect->Open();
	HRESULT hResult;

	IColorFrameSource*colorsource;
	kinect->get_ColorFrameSource(&colorsource);
	IColorFrameReader*colorreader;
	colorsource->OpenReader(&colorreader);

	IBodyFrameSource*bodysource;
	kinect->get_BodyFrameSource(&bodysource);
	IBodyFrameReader*bodyreader;
	bodysource->OpenReader(&bodyreader);

	ICoordinateMapper* coordinatemapper;
	kinect->get_CoordinateMapper(&coordinatemapper);

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

	// 用来显示状态的文本
	string property[FaceProperty_Count];
	property[0] = "Happy";
	property[1] = "Engaged";
	property[2] = "WearingGlasses";
	property[3] = "LeftEyeClosed";
	property[4] = "RightEyeClosed";
	property[5] = "MouthOpen";
	property[6] = "MouthMoved";
	property[7] = "LookingAway";

	while (1)
	{
		Mat asd(1080, 1920, CV_8UC4);

		IColorFrame* colorframe = nullptr;
		hResult = colorreader->AcquireLatestFrame(&colorframe);
		if (colorframe == nullptr)
			continue;
		if (SUCCEEDED(hResult)) {
			colorframe->CopyConvertedFrameDataToArray(1920 * 1080 * 4, reinterpret_cast<BYTE*>(asd.data), ColorImageFormat::ColorImageFormat_Bgra);
		}
		SafeRelease(colorframe);

		IBodyFrame* bodyframe = nullptr;
		hResult = bodyreader->AcquireLatestFrame(&bodyframe);
		if (SUCCEEDED(hResult))
		{
			IBody* body[BODY_COUNT] = { 0 };
			hResult = bodyframe->GetAndRefreshBodyData(BODY_COUNT, body);
			if (SUCCEEDED(hResult))
			{
#pragma omp parallel for 
				for (int i = 0; i < BODY_COUNT; i++)
				{
					BOOLEAN tracked = false;
					hResult = body[i]->get_IsTracked(&tracked);
					if (SUCCEEDED(hResult) && tracked)
					{
						UINT64 trackingId = _UI64_MAX;
						hResult = body[i]->get_TrackingId(&trackingId);
						if (SUCCEEDED(hResult)) {
							facesource[i]->put_TrackingId(trackingId);
							cout << "追踪的人脸ID: " << trackingId << endl;
						}
					}
				}
			}
			for (int i = 0; i < BODY_COUNT; i++)
			{
				SafeRelease(body[i]);
			}
		}
		SafeRelease(bodyframe);
#pragma omp parallel for 
		for (int i = 0; i < BODY_COUNT; i++)
		{
			IFaceFrame*faceframe = nullptr;
			hResult = facereader[i]->AcquireLatestFrame(&faceframe);
			if (faceframe == nullptr)
				continue;
			if (SUCCEEDED(hResult) && faceframe != nullptr)
			{
				BOOLEAN tracked = false;
				hResult = faceframe->get_IsTrackingIdValid(&tracked);
				if (SUCCEEDED(hResult) && tracked)
				{
					IFaceFrameResult *faceresult = nullptr;
					hResult = faceframe->get_FaceFrameResult(&faceresult);
					if (SUCCEEDED(hResult))
					{
						PointF facepoint[FacePointType_Count];
						hResult = faceresult->GetFacePointsInColorSpace(FacePointType_Count, facepoint);
						if (SUCCEEDED(hResult))
						{
							circle(asd, cv::Point(facepoint[0].X, facepoint[0].Y), 5, Scalar(0, 0, 255, 255), -1, CV_AA); // Eye (Left)
							circle(asd, cv::Point(facepoint[1].X, facepoint[1].Y), 5, Scalar(0, 0, 255, 255), -1, CV_AA); // Eye (Right)
							circle(asd, cv::Point(facepoint[2].X, facepoint[2].Y), 5, Scalar(0, 0, 255, 255), -1, CV_AA); // Nose
							circle(asd, cv::Point(facepoint[3].X, facepoint[3].Y), 5, Scalar(0, 0, 255, 255), -1, CV_AA); // Mouth (Left)
							circle(asd, cv::Point(facepoint[4].X, facepoint[4].Y), 5, Scalar(0, 0, 255, 255), -1, CV_AA); // Mouth (Right)
						}
						RectI box;
						hResult = faceresult->get_FaceBoundingBoxInColorSpace(&box);
						if (SUCCEEDED(hResult))
						{
							cv::rectangle(asd, cv::Rect(box.Left, box.Top, box.Right - box.Left, box.Bottom - box.Top), Scalar(0, 0, 255, 255));
						}

						// 四元数 重点来了
						vector<string> result;
						Vector4 faceRotation;
						hResult = faceresult->get_FaceRotationQuaternion(&faceRotation);
						if (SUCCEEDED(hResult))
						{
							// 把四元数分解成三个方向的位置信息
							int pitch, yaw, roll;
							ExtractFaceRotationInDegrees(&faceRotation, &pitch, &yaw, &roll);
							result.push_back("Pitch, Yaw, Roll : " + std::to_string(pitch) + ", " + std::to_string(yaw) + ", " + std::to_string(roll));
						}

						// 面部各种信息
						DetectionResult faceProperty[FaceProperty::FaceProperty_Count];
						hResult = faceresult->GetFaceProperties(FaceProperty::FaceProperty_Count, faceProperty);
						if (SUCCEEDED(hResult))
						{
							for (int count = 0; count < FaceProperty::FaceProperty_Count; count++)
							{
								switch (faceProperty[count])
								{
								case DetectionResult::DetectionResult_Unknown:
									result.push_back(property[count] + " : Unknown");
									break;
								case DetectionResult::DetectionResult_Yes:
									result.push_back(property[count] + " : Yes");
									break;
								case DetectionResult::DetectionResult_No:
									result.push_back(property[count] + " : No");
									break;
								case DetectionResult::DetectionResult_Maybe:
									result.push_back(property[count] + " : Mayby");
									break;
								default:
									break;
								}
							}
						}

						if (box.Left && box.Bottom)
						{
							int offset = 30;
							// 显示字符串
							for (int i = 0; i < 8; i++, offset += 30)
							{
								putText(asd, result[i], cv::Point(box.Left, box.Bottom + offset), FONT_HERSHEY_COMPLEX, 1.0f, Scalar(0, 0, 255, 255), 2, CV_AA);
							}
						}

					}
					SafeRelease(faceresult);
				}
			}
			SafeRelease(faceframe);
		}

		Mat faceimg;
		cv::resize(asd, faceimg, cv::Size(), 0.5, 0.5);
		cv::imshow("Face", faceimg);

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