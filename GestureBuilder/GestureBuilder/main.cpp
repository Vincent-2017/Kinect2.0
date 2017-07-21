#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"
#include <iostream>
#include <Kinect.h>  
#include <Kinect.Face.h>
#include <Kinect.VisualGestureBuilder.h>

#pragma comment ( lib, "kinect20.lib" )  
#pragma comment ( lib, "Kinect20.face.lib" )  
#pragma comment ( lib, "Kinect20.VisualGestureBuilder.lib" )  

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

int main()
{
	IKinectSensor* kinect;
	HRESULT hResult = S_OK;
	GetDefaultKinectSensor(&kinect);
	hResult = kinect->Open();

	IColorFrameSource* colorsource;
	kinect->get_ColorFrameSource(&colorsource);
	IColorFrameReader* colorreader;
	colorsource->OpenReader(&colorreader);

	IBodyFrameSource* bodysource;
	kinect->get_BodyFrameSource(&bodysource);
	IBodyFrameReader* bodyreader;
	bodysource->OpenReader(&bodyreader);

	IFrameDescription* dc;
	colorsource->get_FrameDescription(&dc);
	int width = 0;
	int height = 0;
	dc->get_Width(&width); // 1920
	dc->get_Height(&height); // 1080
	unsigned int buffersize = width * height * 4 * sizeof(unsigned char);

	Mat bufferMat(height, width, CV_8UC4);
	Mat bodyMat(height / 2, width / 2, CV_8UC4);
	namedWindow("Gesture");

	ICoordinateMapper* coordinate;
	kinect->get_CoordinateMapper(&coordinate);

	// 姿势数据源
	IVisualGestureBuilderFrameSource* gesturesource[BODY_COUNT];
	IVisualGestureBuilderFrameReader* gesturereader[BODY_COUNT];
	for (int count = 0; count < BODY_COUNT; count++)
	{
		CreateVisualGestureBuilderFrameSource(kinect, 0, &gesturesource[count]);

		gesturesource[count]->OpenReader(&gesturereader[count]);
	}

	// 读取样本
	IVisualGestureBuilderDatabase* gesturedatabase;
	CreateVisualGestureBuilderDatabaseInstanceFromFile(L"wave.gba", &gesturedatabase);

	// 样本库中的姿势和数目
	UINT gesturecount = 0;
	gesturedatabase->get_AvailableGesturesCount(&gesturecount);

	IGesture* gesture;
	hResult = gesturedatabase->get_AvailableGestures(gesturecount, &gesture);
	// 添加姿势到数据源中，并使能
	if (SUCCEEDED(hResult) && gesture != nullptr)
	{
		for (int count = 0; count < BODY_COUNT; count++)
		{
			gesturesource[count]->AddGesture(gesture);

			gesturesource[count]->SetIsEnabled(gesture, true);
		}
	}

	while (1)
	{
		// 获得色彩数据并释放对象
		IColorFrame* colorframe = nullptr;
		hResult = colorreader->AcquireLatestFrame(&colorframe);
		if (SUCCEEDED(hResult))
		{
			hResult = colorframe->CopyConvertedFrameDataToArray(buffersize, reinterpret_cast<BYTE*>(bufferMat.data), ColorImageFormat::ColorImageFormat_Bgra);
			if (SUCCEEDED(hResult))
			{
				resize(bufferMat, bodyMat, Size(), 0.5, 0.5);
			}
		}
		SafeRelease(colorframe);

		// 骨骼帧
		IBodyFrame* bodyframe = nullptr;
		hResult = bodyreader->AcquireLatestFrame(&bodyframe);
		if (SUCCEEDED(hResult))
		{
			IBody* body[BODY_COUNT] = { 0 };
			hResult = bodyframe->GetAndRefreshBodyData(BODY_COUNT, body);
			if (SUCCEEDED(hResult))
			{
				for (int count = 0; count < BODY_COUNT; count++)
				{
					// 检测骨骼被追踪
					BOOLEAN istracked = false;
					hResult = body[count]->get_IsTracked(&istracked);
					if (SUCCEEDED(hResult) && istracked)
					{
						Joint joint[JointType::JointType_Count];
						hResult = body[count]->GetJoints(JointType::JointType_Count, joint);
						if (SUCCEEDED(hResult))
						{
							for (int type = 0; type < JointType::JointType_Count; type++)
							{
								ColorSpacePoint colorspacepoint = { 0 };
								coordinate->MapCameraPointToColorSpace(joint[type].Position, &colorspacepoint);
								int x = static_cast<int>(colorspacepoint.X);
								int y = static_cast<int>(colorspacepoint.Y);
								if ((x >= 0) && (x < width) && (y >= 0) && (y < height))
								{
									circle(bufferMat, Point(x, y), 5, Scalar(0, 0, 255, 0), 10);
								}
							}
						}
						UINT64 trackingId = _UI64_MAX;
						hResult = body[count]->get_TrackingId(&trackingId);
						if (SUCCEEDED(hResult))
						{
							gesturesource[count]->put_TrackingId(trackingId);
							cout << "追踪的骨骼ID：" << trackingId << endl;
						}
					}
				}
				resize(bufferMat, bodyMat, Size(), 0.5, 0.5);
			}
			for (int count = 0; count < BODY_COUNT; count++)
			{
				SafeRelease(body[count]);
			}
		}
		SafeRelease(bodyframe);


		for (int count = 0; count < BODY_COUNT; count++)
		{
			IVisualGestureBuilderFrame* gestureframe = nullptr;
			hResult = gesturereader[count]->CalculateAndAcquireLatestFrame(&gestureframe);
			if (SUCCEEDED(hResult) && gestureframe != nullptr)
			{
				BOOLEAN bGestureTracked = false;
				hResult = gestureframe->get_IsTrackingIdValid(&bGestureTracked);
				if (SUCCEEDED(hResult) && bGestureTracked)
				{
					//静态动作
					IDiscreteGestureResult* gestureresult = nullptr;
					hResult = gestureframe->get_DiscreteGestureResult(gesture, &gestureresult);
					if (SUCCEEDED(hResult) && gestureresult != nullptr)
					{
						BOOLEAN bDetected = false;
						hResult = gestureresult->get_Detected(&bDetected);
						if (SUCCEEDED(hResult) && bDetected)
						{
							cout << "Detected Gesture" << endl;
						}
					}

					SafeRelease(gestureresult);
				}
			}
			SafeRelease(gestureframe);
		}

		imshow("Gesture", bodyMat);

		if (waitKey(10) == VK_ESCAPE)
		{
			break;
		}
	}

	SafeRelease(colorsource);
	SafeRelease(bodysource);
	SafeRelease(colorreader);
	SafeRelease(bodyreader);
	SafeRelease(dc);
	SafeRelease(coordinate);
	for (int count = 0; count < BODY_COUNT; count++)
	{
		SafeRelease(gesturesource[count]);
		SafeRelease(gesturereader[count]);
	}
	SafeRelease(gesturedatabase);
	SafeRelease(gesture);
	if (kinect)
	{
		kinect->Close();
	}
	SafeRelease(kinect);

	return 0;
}