#include "global.h"

bool IsSkeletonTrackedWell(Joint & shoulder, Joint & elbow, Joint & hand, ICoordinateMapper * myMapper);

Point   p_s, p_e, p_h;
int data[10][2] , j = 0;

int main(void)
{
	IKinectSensor   * mySensor = nullptr;
	GetDefaultKinectSensor(&mySensor);
	mySensor->Open();

	IColorFrameSource   * myColorSource = nullptr;
	mySensor->get_ColorFrameSource(&myColorSource);

	IColorFrameReader   * myColorReader = nullptr;
	myColorSource->OpenReader(&myColorReader);

	int colorHeight = 0, colorWidth = 0;
	IFrameDescription   * myDescription = nullptr;
	myColorSource->get_FrameDescription(&myDescription);
	myDescription->get_Height(&colorHeight);
	myDescription->get_Width(&colorWidth);

	IColorFrame * myColorFrame = nullptr;
	Mat original(colorHeight, colorWidth, CV_8UC4);

	//**********************以上为ColorFrame的读取前准备**************************

	IBodyFrameSource    * myBodySource = nullptr;
	mySensor->get_BodyFrameSource(&myBodySource);

	IBodyFrameReader    * myBodyReader = nullptr;
	myBodySource->OpenReader(&myBodyReader);

	int myBodyCount = 0;
	myBodySource->get_BodyCount(&myBodyCount);

	IBodyFrame  * myBodyFrame = nullptr;

	ICoordinateMapper   * myMapper = nullptr;
	mySensor->get_CoordinateMapper(&myMapper);

	//**********************以上为BodyFrame以及Mapper的准备***********************
	while (1)
	{
		while (myColorReader->AcquireLatestFrame(&myColorFrame) != S_OK);
		myColorFrame->CopyConvertedFrameDataToArray(colorHeight * colorWidth * 4, original.data, ColorImageFormat_Bgra); // 图像大小、目标地址、图像格式
		Mat copy = original.clone();        //读取彩色图像并输出到矩阵

		while (myBodyReader->AcquireLatestFrame(&myBodyFrame) != S_OK); //读取身体图像
		IBody   **  myBodyArr = new IBody *[myBodyCount];       //为存身体数据的数组做准备
		for (int i = 0; i < myBodyCount; i++)
			myBodyArr[i] = nullptr;

		if (myBodyFrame->GetAndRefreshBodyData(myBodyCount, myBodyArr) == S_OK)     //把身体数据输入数组
		for (int i = 0; i < myBodyCount; i++)
		{
			BOOLEAN     result = false;
			if (myBodyArr[i]->get_IsTracked(&result) == S_OK && result) //先判断是否侦测到
			{
				Joint   myJointArr[JointType_Count];
				if (myBodyArr[i]->GetJoints(JointType_Count, myJointArr) == S_OK)   //如果侦测到就把关节数据输入到数组并画图
				{
					DrawBody(copy, myJointArr, myMapper);
					drawhandstate(copy, myJointArr[JointType_HandLeft], myJointArr[JointType_HandRight], myBodyArr[i], myMapper);
					IsSkeletonTrackedWell(myJointArr[JointType_ShoulderRight], myJointArr[JointType_ElbowRight], myJointArr[JointType_HandRight], myMapper);
				}
			}
		}
		delete[]myBodyArr;
		myBodyFrame->Release();
		myColorFrame->Release();

		imshow("TEST", copy);
		if (waitKey(30) == VK_ESCAPE)
			break;
	}
	myMapper->Release();

	myDescription->Release();
	myColorReader->Release();
	myColorSource->Release();

	myBodyReader->Release();
	myBodySource->Release();
	mySensor->Close();
	mySensor->Release();

	return  0;
}

// 判断骨骼追踪情况：包括骨骼追踪完好且手部位置在肘上面  
bool IsSkeletonTrackedWell(Joint & shoulder, Joint & elbow, Joint & hand, ICoordinateMapper * myMapper)
{
	if (shoulder.TrackingState == TrackingState_Tracked && elbow.TrackingState == TrackingState_Tracked && hand.TrackingState == TrackingState_Tracked)
	{
		ColorSpacePoint t_point;    //要把关节点用的摄像机坐标下的点转换成彩色空间的点
		myMapper->MapCameraPointToColorSpace(shoulder.Position, &t_point);
		p_s.x = t_point.X;
		p_s.y = t_point.Y;
		myMapper->MapCameraPointToColorSpace(elbow.Position, &t_point);
		p_e.x = t_point.X;
		p_e.y = t_point.Y;
		myMapper->MapCameraPointToColorSpace(hand.Position, &t_point);
		p_h.x = t_point.X;
		p_h.y = t_point.Y;
		if (p_h.y < p_e.y )
		{
			int elbow_range = 0 , hand_range = 0;
			int elbow_MAX = INT_MIN, elbow_MIN = INT_MAX, hand_MAX = INT_MIN, hand_MIN = INT_MAX;
			int LeftCo = 0, RightCo = 0;
			for (int i = 0; i < 9; i++)
			{
				data[i][0] = data[i + 1][0];
				data[i][1] = data[i + 1][1];				
			}
			data[9][0] = p_e.x;
			data[9][1] = p_h.x;
			for (int i = 0; i < 10; i++)
			{
				if (data[i][0]>elbow_MAX) elbow_MAX = data[i][0];
				if (data[i][0]<elbow_MIN) elbow_MIN = data[i][0];
				if (data[i][1]>hand_MAX) hand_MAX = data[i][1];
				if (data[i][1]<hand_MIN) hand_MIN = data[i][1];
				if (data[i][1] < data[i][0]) LeftCo++;
				else if (data[i][1] > data[i][0]) RightCo++;
				else;
				
			}
			elbow_range = elbow_MAX - elbow_MIN ;
			hand_range = hand_MAX - hand_MIN;
			cout << "R_e:" << p_e.x << "," << p_e.y << "  手肘运动范围：" << elbow_range << endl;
			cout << "R_h:" << p_h.x << "," << p_h.y << "  手掌运动范围：" << hand_range << endl;
			if (hand_range >= 250 && elbow_range <= 100 )
				cout << "识别到挥手动作" << endl;
			else cout << "未识别" << endl;
			return true;
		}
		else
		{
			cout << "不满足手掌高于手肘条件" << endl;
			return false;
		}
	}
	else return false;
}
