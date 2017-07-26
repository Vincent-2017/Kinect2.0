#include "global.h"

bool GestureDetection(Joint & elbow, Joint & hand, ICoordinateMapper * myMapper);

int data[10][2] ;

int main(void)
{
	// 获得Kinect传感器
	IKinectSensor   * mySensor = nullptr;
	GetDefaultKinectSensor(&mySensor);
	// 打开Kinect
	mySensor->Open();

	//**********************ColorFrame的读取前准备**************************
	// 彩色数据源
	IColorFrameSource   * myColorSource = nullptr;
	mySensor->get_ColorFrameSource(&myColorSource);
	IColorFrameReader   * myColorReader = nullptr;
	myColorSource->OpenReader(&myColorReader);
	// 获得彩色图像大小
	int colorHeight = 0, colorWidth = 0;
	IFrameDescription   * myDescription = nullptr;
	myColorSource->get_FrameDescription(&myDescription);
	myDescription->get_Height(&colorHeight);
	myDescription->get_Width(&colorWidth);
	// 彩色帧
	IColorFrame * myColorFrame = nullptr;
	Mat original(colorHeight, colorWidth, CV_8UC4);

	//**********************BodyFrame以及Mapper的准备***********************
	// 骨骼数据源
	IBodyFrameSource    * myBodySource = nullptr;
	mySensor->get_BodyFrameSource(&myBodySource);
	IBodyFrameReader    * myBodyReader = nullptr;
	myBodySource->OpenReader(&myBodyReader);
	// 获得骨骼数量
	int myBodyCount = 0;
	myBodySource->get_BodyCount(&myBodyCount);
	// 骨骼帧
	IBodyFrame  * myBodyFrame = nullptr;
	// 映射坐标系
	ICoordinateMapper   * myMapper = nullptr;
	mySensor->get_CoordinateMapper(&myMapper);

	// 主程序
	while (1)
	{
		// 读取彩色图像
		while (myColorReader->AcquireLatestFrame(&myColorFrame) != S_OK);
		// 将图像帧拷贝到数组中 Bgra格式
		myColorFrame->CopyConvertedFrameDataToArray(colorHeight * colorWidth * 4, original.data, ColorImageFormat_Bgra); // 图像大小、目标地址、图像格式
		// 读取彩色图像并输出到矩阵
		Mat copy = original.clone();   

		// 读取骨骼图像
		while (myBodyReader->AcquireLatestFrame(&myBodyFrame) != S_OK); 
		// 为存身体数据的数组做准备，二维数组，一维是身体数目，二维是关节数目
		IBody   **  myBodyArr = new IBody *[myBodyCount]; 
		// 身体数组初始化
		for (int i = 0; i < myBodyCount; i++)	myBodyArr[i] = nullptr;
		//把身体数据输入数组
		if (myBodyFrame->GetAndRefreshBodyData(myBodyCount, myBodyArr) == S_OK)   

		// 对每一个骨骼进行处理
		for (int i = 0; i < myBodyCount; i++)
		{
			BOOLEAN     result = false;
			//先判断是否侦测到该骨骼
			if (myBodyArr[i]->get_IsTracked(&result) == S_OK && result) 
			{
				Joint   myJointArr[JointType_Count];
				//如果侦测到就把关节数据输入到数组并画图
				if (myBodyArr[i]->GetJoints(JointType_Count, myJointArr) == S_OK)   
				{
					// 绘制身体关节
					DrawBody(copy, myJointArr, myMapper);
					// 绘制手部状态
					drawhandstate(copy, myJointArr[JointType_HandLeft], myJointArr[JointType_HandRight], myBodyArr[i], myMapper);
					// 挥手检测
					GestureDetection(myJointArr[JointType_ElbowRight], myJointArr[JointType_HandRight], myMapper);
				}
			}
		}
		// 内存回收
		delete[]myBodyArr;
		// 这两帧必须在while(1)的末尾释放，为下一次的数据获取做准备
		myBodyFrame->Release();
		myColorFrame->Release();

		imshow("彩色骨骼挥手识别", copy);
		if (waitKey(1) == 'q') 
			imwrite("1.jpg", copy);
		if (waitKey(30) == VK_ESCAPE)	
			break;
	}

	// 与程序开始相反，后建立的先释放
	myMapper->Release();

	myColorFrame->Release();
	myDescription->Release();
	myColorReader->Release();
	myColorSource->Release();

	myBodyFrame->Release();
	myBodyReader->Release();
	myBodySource->Release();

	mySensor->Close();
	mySensor->Release();

	return  0;
}

// 判断骨骼追踪情况：包括骨骼追踪完好且手部位置在肘上面  
bool GestureDetection(Joint & elbow, Joint & hand, ICoordinateMapper * myMapper)
{
	// 骨骼追踪完好
	if (elbow.TrackingState == TrackingState_Tracked && hand.TrackingState == TrackingState_Tracked)
	{
		// 要把关节点用的摄像机坐标下的点转换成彩色空间的点
		ColorSpacePoint t_point;    
		Point	p_e, p_h;
		myMapper->MapCameraPointToColorSpace(elbow.Position, &t_point);
		p_e.x = t_point.X;
		p_e.y = t_point.Y;
		myMapper->MapCameraPointToColorSpace(hand.Position, &t_point);
		p_h.x = t_point.X;
		p_h.y = t_point.Y;
		// 手掌高于手肘
		if (p_h.y < p_e.y)
		{
			int elbow_range = 0, hand_range = 0;
			int elbow_MAX = INT_MIN, elbow_MIN = INT_MAX, hand_MAX = INT_MIN, hand_MIN = INT_MAX;
			// 保存近10帧的手掌和手肘位置，并进行数据更新
			for (int i = 0; i < 9; i++)
			{
				data[i][0] = data[i + 1][0];
				data[i][1] = data[i + 1][1];
			}
			data[9][0] = p_e.x;
			data[9][1] = p_h.x;
			// 计算手掌和手肘的运动范围
			for (int i = 0; i < 10; i++)
			{
				if (data[i][0]>elbow_MAX) elbow_MAX = data[i][0];
				if (data[i][0]<elbow_MIN) elbow_MIN = data[i][0];
				if (data[i][1]>hand_MAX) hand_MAX = data[i][1];
				if (data[i][1] < hand_MIN) hand_MIN = data[i][1];
			}
			elbow_range = elbow_MAX - elbow_MIN;
			hand_range = hand_MAX - hand_MIN;
			cout << "R_e:" << p_e.x << "," << p_e.y << "  手肘运动范围：" << elbow_range << endl;
			cout << "R_h:" << p_h.x << "," << p_h.y << "  手掌运动范围：" << hand_range << endl;
			// 手肘、手掌运动阈值判定
			if (hand_range >= 250 && elbow_range <= 100)
			{
				cout << "识别到挥手动作" << endl;
				return true;
			}
			else //cout << "未识别" << endl;
				return false;
		}
		else
		{
			//cout << "不满足手掌高于手肘条件" << endl;
			return false;
		}
	}
	else return false;
}
