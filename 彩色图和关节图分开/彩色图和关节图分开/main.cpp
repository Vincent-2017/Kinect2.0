#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"
#include <iostream>
#include <Kinect.h>  

using namespace cv;
using namespace std;

// 安全释放指针
template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL)
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}

void DrawBone(JointType b, JointType c, ICoordinateMapper*coordinatemapper, Joint joint[], Mat&a)
{
	DepthSpacePoint d1, d2;
	coordinatemapper->MapCameraPointToDepthSpace(joint[b].Position, &d1);
	coordinatemapper->MapCameraPointToDepthSpace(joint[c].Position, &d2);
	if (d1.X>0 && d1.X<512 && d1.Y>0 && d1.Y<424 && d2.X>0 && d2.X<512 && d2.Y>0 && d2.Y<424)
		line(a, Point(d1.X, d1.Y), Point(d2.X, d2.Y), Scalar(0, 255, 0, 255), 3);
	else
		line(a, Point(d1.X, d1.Y), Point(d2.X, d2.Y), Scalar(255, 255, 255, 255), 1);
	circle(a, Point(d1.X, d1.Y), 2, Scalar(255, 255, 255, 255), 4);
	circle(a, Point(d2.X, d2.Y), 2, Scalar(255, 255, 255, 255), 4);
}

int main()
{
	HRESULT hResult = S_OK;     //用于检测操作是否成功
	IKinectSensor *kinect;           //创建一个感应器
	hResult = GetDefaultKinectSensor(&kinect);
	kinect->Open();     //打开感应器
	if (FAILED(hResult))
	{
		return hResult;
	}

	IColorFrameSource *colorsource;
	kinect->get_ColorFrameSource(&colorsource); // 显示数据源的源数据并为阅读器（readers）提供途径、为传感器每个数据提供一种数据源
	IColorFrameReader *colorreader;
	colorsource->OpenReader(&colorreader); // 提供获取帧的途径（事件机制、轮询机制）
	IFrameDescription *colorde;
	colorsource->get_FrameDescription(&colorde); // 发送帧事件参数，获取最近一帧的数据
	int width = 0;      //长和宽
	int hight = 0;
	colorde->get_Height(&hight);
	colorde->get_Width(&width);
	Mat a(hight, width, CV_8UC4);      //注意：这里必须为4通道的图，Kinect的数据只能以Bgra格式传出
	namedWindow("aaa");

	IBodyFrameSource*bodysource;
	kinect->get_BodyFrameSource(&bodysource);
	IBodyFrameReader*bodyreader;
	bodysource->OpenReader(&bodyreader);
	ICoordinateMapper* coordinatemapper;
	kinect->get_CoordinateMapper(&coordinatemapper);

	while (1)
	{
		IColorFrame*frame;
		hResult = colorreader->AcquireLatestFrame(&frame);
		if (SUCCEEDED(hResult))
		{
			frame->CopyConvertedFrameDataToArray(hight*width * 4, reinterpret_cast<BYTE*>(a.data), ColorImageFormat::ColorImageFormat_Bgra);   //传出数据
		}
		if (frame != NULL)   //释放
		{
			frame->Release();
			frame = NULL;
		}
		if (waitKey(30) == VK_ESCAPE)
			break;
		imshow("aaa", a);

		Mat asd(424, 512, CV_8UC4);
		IBodyFrame* bodyframe = nullptr;
		hResult = bodyreader->AcquireLatestFrame(&bodyframe);
		if (SUCCEEDED(hResult))
		{
			IBody* body[BODY_COUNT] = { 0 };
			hResult = bodyframe->GetAndRefreshBodyData(BODY_COUNT, body);
			if (SUCCEEDED(hResult))
			{
				for (int i = 0; i < BODY_COUNT; i++)
				{
					BOOLEAN tracked = false;
					hResult = body[i]->get_IsTracked(&tracked);
					if (SUCCEEDED(hResult) && tracked)
					{

						Joint joint[JointType_Count];
						hResult = body[i]->GetJoints(JointType_Count, joint);
						DepthSpacePoint depthspacepoint;

						if (SUCCEEDED(hResult))
						{
							// Torso
							DrawBone(JointType_Head, JointType_Neck, coordinatemapper, joint, asd);
							DrawBone(JointType_Neck, JointType_SpineShoulder, coordinatemapper, joint, asd);
							DrawBone(JointType_SpineShoulder, JointType_SpineMid, coordinatemapper, joint, asd);
							DrawBone(JointType_SpineMid, JointType_SpineBase, coordinatemapper, joint, asd);
							DrawBone(JointType_SpineShoulder, JointType_ShoulderRight, coordinatemapper, joint, asd);
							DrawBone(JointType_SpineShoulder, JointType_ShoulderLeft, coordinatemapper, joint, asd);
							DrawBone(JointType_SpineBase, JointType_HipRight, coordinatemapper, joint, asd);
							DrawBone(JointType_SpineBase, JointType_HipLeft, coordinatemapper, joint, asd);

							// Right Arm    
							DrawBone(JointType_ShoulderRight, JointType_ElbowRight, coordinatemapper, joint, asd);
							DrawBone(JointType_ElbowRight, JointType_WristRight, coordinatemapper, joint, asd);
							DrawBone(JointType_WristRight, JointType_HandRight, coordinatemapper, joint, asd);
							DrawBone(JointType_HandRight, JointType_HandTipRight, coordinatemapper, joint, asd);
							DrawBone(JointType_WristRight, JointType_ThumbRight, coordinatemapper, joint, asd);

							// Left Arm
							DrawBone(JointType_ShoulderLeft, JointType_ElbowLeft, coordinatemapper, joint, asd);
							DrawBone(JointType_ElbowLeft, JointType_WristLeft, coordinatemapper, joint, asd);
							DrawBone(JointType_WristLeft, JointType_HandLeft, coordinatemapper, joint, asd);
							DrawBone(JointType_HandLeft, JointType_HandTipLeft, coordinatemapper, joint, asd);
							DrawBone(JointType_WristLeft, JointType_ThumbLeft, coordinatemapper, joint, asd);

							// Right Leg
							DrawBone(JointType_HipRight, JointType_KneeRight, coordinatemapper, joint, asd);
							DrawBone(JointType_KneeRight, JointType_AnkleRight, coordinatemapper, joint, asd);
							DrawBone(JointType_AnkleRight, JointType_FootRight, coordinatemapper, joint, asd);

							// Left Leg
							DrawBone(JointType_HipLeft, JointType_KneeLeft, coordinatemapper, joint, asd);
							DrawBone(JointType_KneeLeft, JointType_AnkleLeft, coordinatemapper, joint, asd);
							DrawBone(JointType_AnkleLeft, JointType_FootLeft, coordinatemapper, joint, asd);

							DepthSpacePoint d1, d2;
							coordinatemapper->MapCameraPointToDepthSpace(joint[JointType_HandLeft].Position, &d1);
							coordinatemapper->MapCameraPointToDepthSpace(joint[JointType_HandRight].Position, &d2);
							HandState left;
							body[i]->get_HandLeftState(&left);
							HandState right;
							body[i]->get_HandRightState(&right);
							switch (left)
							{
							case HandState_Closed:
								circle(asd, Point(d1.X, d1.Y), 10, Scalar(0, 0, 255, 1), 20); break;
							case HandState_Open:
								circle(asd, Point(d1.X, d1.Y), 10, Scalar(0, 255, 0, 1), 20); break;
							case HandState_Lasso:
								circle(asd, Point(d1.X, d1.Y), 10, Scalar(255, 0, 0, 1), 20); break;
							default:
								break;
							}
							switch (right)
							{
							case HandState_Closed:
								circle(asd, Point(d2.X, d2.Y), 10, Scalar(0, 0, 255, 1), 20); break;
							case HandState_Open:
								circle(asd, Point(d2.X, d2.Y), 10, Scalar(0, 255, 0, 1), 20); break;
							case HandState_Lasso:
								circle(asd, Point(d2.X, d2.Y), 10, Scalar(255, 0, 0, 1), 20); break;
							default:
								break;
							}
						}
					}
				}
			}
			for (int count = 0; count < BODY_COUNT; count++)
			{
				SafeRelease(body[count]);
			}
		}

		SafeRelease(bodyframe);

		imshow("aaaaaaa", asd);
		if (waitKey(33) == VK_ESCAPE)
		{
			break;
		}
	}
	if (colorsource != NULL)    //全部释放掉
	{
		colorsource->Release();
		colorsource = NULL;
	}
	if (colorreader != NULL)
	{
		colorreader->Release();
		colorreader = NULL;
	}
	if (colorde != NULL)
	{
		colorde->Release();
		colorde = NULL;
	}
	if (kinect)
	{
		kinect->Close();
	}
	if (kinect != NULL)
	{
		kinect->Release();
		kinect = NULL;
	}

	SafeRelease(bodysource);
	SafeRelease(bodyreader);
	SafeRelease(coordinatemapper);
	if (kinect) {
		kinect->Close();
	}
	SafeRelease(kinect);
	destroyAllWindows();
}



