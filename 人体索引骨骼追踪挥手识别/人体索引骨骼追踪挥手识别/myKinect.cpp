#include "myKinect.h"
#include <iostream>

int myBodyCount;
int data[10][2];

// Initializes the default Kinect sensor
HRESULT CBodyBasics::InitializeDefaultSensor()
{
	// 用于判断每次读取操作的成功与否
	HRESULT hr;
	// 搜索kinect
	GetDefaultKinectSensor(&m_pKinectSensor);
	// 找到kinect设备
	if (m_pKinectSensor)
	{
		// 打开kinect
		hr = m_pKinectSensor->Open();
		// 定义两类数据源
		IBodyFrameSource* pBodyFrameSource = NULL;
		IBodyIndexFrameSource* pBodyIndexFrameSource = NULL;
		// 获取骨架数据源
		if (SUCCEEDED(hr))	hr = m_pKinectSensor->get_BodyFrameSource(&pBodyFrameSource);
		if (SUCCEEDED(hr))	hr = pBodyFrameSource->OpenReader(&m_pBodyFrameReader);
		// 获得骨骼数量
		pBodyFrameSource->get_BodyCount(&myBodyCount);
		// 获取人物索引图数据源
		if (SUCCEEDED(hr))	hr = m_pKinectSensor->get_BodyIndexFrameSource(&pBodyIndexFrameSource);
		if (SUCCEEDED(hr))	hr = pBodyIndexFrameSource->OpenReader(&m_pBodyIndexFrameReader);
		// Source使用完毕，释放数据源
		SafeRelease(pBodyFrameSource);
		SafeRelease(pBodyIndexFrameSource);
		// 坐标映射
		if (SUCCEEDED(hr))	hr = m_pKinectSensor->get_CoordinateMapper(&m_pCoordinateMapper);
	}

	if (!m_pKinectSensor || FAILED(hr))
	{
		std::cout << "Kinect initialization failed!" << std::endl;
		return E_FAIL;
	}

	//用于画骨架、人物索引图的MAT
	IndexImage.create(cDepthHeight, cDepthWidth, CV_8UC3);
	// 初始化0
	IndexImage.setTo(0); 

	return hr;
}


// Main processing function
void CBodyBasics::Update()
{
	//每次先清空skeletonImg
	IndexImage.setTo(0);
	//如果丢失了kinect，则不继续操作
	if (!m_pBodyFrameReader)	return;
	// 数据帧
	IBodyFrame* pBodyFrame = NULL;//骨架信息
	IBodyIndexFrame* pBodyIndexFrame = NULL;//人物索引图
	//记录每次操作的成功与否
	HRESULT hr = S_OK;
	//---------------------------------------获取人物索引图并显示---------------------------------
	//获得人物索引图
	if (SUCCEEDED(hr))	hr = m_pBodyIndexFrameReader->AcquireLatestFrame(&pBodyIndexFrame);
	if (SUCCEEDED(hr))
	{
		//人物索引图是8位uchar，有人是黑色，没人是白色
		BYTE *bodyIndexArray = new BYTE[cDepthHeight * cDepthWidth];
		// 拷贝索引图到数组 bodyIndexArray
		pBodyIndexFrame->CopyFrameDataToArray(cDepthHeight * cDepthWidth, bodyIndexArray);
		//把索引图画到MAT里 ，指针 IndexData 指向数组 IndexImage 的地址
		uchar* IndexData = (uchar*)IndexImage.data;
		for (int j = 0; j < cDepthHeight * cDepthWidth; ++j){
			*IndexData = bodyIndexArray[j]; ++IndexData;
			*IndexData = bodyIndexArray[j]; ++IndexData;
			*IndexData = bodyIndexArray[j]; ++IndexData;
		}
		// 回收内存
		delete[] bodyIndexArray; 
	}
	//必须要释放，否则之后无法获得新的frame数据
	SafeRelease(pBodyIndexFrame);

	//-----------------------------获取骨架并显示----------------------------
	//获取骨架信息
	if (SUCCEEDED(hr))	hr = m_pBodyFrameReader->AcquireLatestFrame(&pBodyFrame);
	if (SUCCEEDED(hr))
	{
		// 每一个IBody可以追踪一个人，总共可以追踪六个人 . 二维数组，一维是身体数目，二维是关节数目
		IBody   **  ppBodies = new IBody *[myBodyCount];
		// 身体数组初始化
		for (int i = 0; i < myBodyCount; i++)	ppBodies[i] = nullptr;
		//把kinect追踪到的人的信息，分别存到每一个IBody中
		hr = pBodyFrame->GetAndRefreshBodyData(myBodyCount, ppBodies);

		//对每一个IBody，我们找到他的骨架信息，并且画出来
		if (SUCCEEDED(hr))	ProcessBody(myBodyCount, ppBodies);

		// 释放所有身体内存
		for (int i = 0; i < myBodyCount; ++i)	SafeRelease(ppBodies[i]);
	}
	SafeRelease(pBodyFrame);//必须要释放，否则之后无法获得新的frame数据
}

// Handle new body data
void CBodyBasics::ProcessBody(int nBodyCount, IBody** ppBodies)  // ppBodies 二维数组，一维是身体数目，二维是关节数目
{
	//记录操作结果是否成功
	HRESULT hr;

	//对于每一个IBody
	for (int i = 0; i < nBodyCount; ++i)
	{
		IBody* pBody = ppBodies[i];
		if (pBody) //还没有搞明白这里pBody和下面的bTracked有什么区别
		{
			BOOLEAN bTracked = false;
			// 身体被追踪到？？
			hr = pBody->get_IsTracked(&bTracked);
			if (SUCCEEDED(hr) && bTracked)
			{
				Joint joints[JointType_Count];//存储关节点类
				HandState leftHandState = HandState_Unknown;//左手状态
				HandState rightHandState = HandState_Unknown;//右手状态

				//获取左右手状态
				pBody->get_HandLeftState(&leftHandState);
				pBody->get_HandRightState(&rightHandState);

				//存储深度坐标系中的关节点位置
				DepthSpacePoint *depthSpacePosition = new DepthSpacePoint[JointType_Count];

				//获得关节点类
				hr = pBody->GetJoints(JointType_Count, joints);
				if (SUCCEEDED(hr))
				{
					for (int j = 0; j < JointType_Count; ++j)
					{
						//将每个关节点坐标从摄像机坐标系（-1~1）转到深度坐标系（424*512）
						m_pCoordinateMapper->MapCameraPointToDepthSpace(joints[j].Position, &depthSpacePosition[j]);
					}

					//------------------------hand state left-------------------------------
					DrawHandState(depthSpacePosition[JointType_HandLeft], leftHandState);
					DrawHandState(depthSpacePosition[JointType_HandRight], rightHandState);

					//---------------------------body-------------------------------
					DrawBone(joints, depthSpacePosition, JointType_Head, JointType_Neck);
					DrawBone(joints, depthSpacePosition, JointType_Neck, JointType_SpineShoulder);
					DrawBone(joints, depthSpacePosition, JointType_SpineShoulder, JointType_SpineMid);
					DrawBone(joints, depthSpacePosition, JointType_SpineMid, JointType_SpineBase);
					DrawBone(joints, depthSpacePosition, JointType_SpineShoulder, JointType_ShoulderRight);
					DrawBone(joints, depthSpacePosition, JointType_SpineShoulder, JointType_ShoulderLeft);
					DrawBone(joints, depthSpacePosition, JointType_SpineBase, JointType_HipRight);
					DrawBone(joints, depthSpacePosition, JointType_SpineBase, JointType_HipLeft);

					// -----------------------Right Arm ------------------------------------ 
					DrawBone(joints, depthSpacePosition, JointType_ShoulderRight, JointType_ElbowRight);
					DrawBone(joints, depthSpacePosition, JointType_ElbowRight, JointType_WristRight);
					DrawBone(joints, depthSpacePosition, JointType_WristRight, JointType_HandRight);
					DrawBone(joints, depthSpacePosition, JointType_HandRight, JointType_HandTipRight);
					DrawBone(joints, depthSpacePosition, JointType_WristRight, JointType_ThumbRight);

					//----------------------------------- Left Arm--------------------------
					DrawBone(joints, depthSpacePosition, JointType_ShoulderLeft, JointType_ElbowLeft);
					DrawBone(joints, depthSpacePosition, JointType_ElbowLeft, JointType_WristLeft);
					DrawBone(joints, depthSpacePosition, JointType_WristLeft, JointType_HandLeft);
					DrawBone(joints, depthSpacePosition, JointType_HandLeft, JointType_HandTipLeft);
					DrawBone(joints, depthSpacePosition, JointType_WristLeft, JointType_ThumbLeft);

					// ----------------------------------Right Leg--------------------------------
					DrawBone(joints, depthSpacePosition, JointType_HipRight, JointType_KneeRight);
					DrawBone(joints, depthSpacePosition, JointType_KneeRight, JointType_AnkleRight);
					DrawBone(joints, depthSpacePosition, JointType_AnkleRight, JointType_FootRight);

					// -----------------------------------Left Leg---------------------------------
					DrawBone(joints, depthSpacePosition, JointType_HipLeft, JointType_KneeLeft);
					DrawBone(joints, depthSpacePosition, JointType_KneeLeft, JointType_AnkleLeft);
					DrawBone(joints, depthSpacePosition, JointType_AnkleLeft, JointType_FootLeft);
				}

				// 挥手检测
				GestureDetection(joints[JointType_ElbowRight], joints[JointType_HandRight], m_pCoordinateMapper);

				// 回收内存
				delete[] depthSpacePosition;
			}
		}
	}
	cv::imshow("IndexImage", IndexImage);
	cv::waitKey(5);
}

//画手的状态
void CBodyBasics::DrawHandState(const DepthSpacePoint depthSpacePosition, HandState handState)
{
	//给不同的手势分配不同颜色
	CvScalar color;
	switch (handState){
	case HandState_Open:
		color = cvScalar(255, 0, 0);
		break;
	case HandState_Closed:
		color = cvScalar(0, 255, 0);
		break;
	case HandState_Lasso:
		color = cvScalar(0, 0, 255);
		break;
	default://如果没有确定的手势，就不要画
		return;
	}

	circle(IndexImage,cvPoint(depthSpacePosition.X, depthSpacePosition.Y),20, color, -1); // 空间坐标大写
}


/// Draws one bone of a body (joint to joint)
void CBodyBasics::DrawBone(const Joint* pJoints, const DepthSpacePoint* depthSpacePosition, JointType joint0, JointType joint1)
{
	TrackingState joint0State = pJoints[joint0].TrackingState;
	TrackingState joint1State = pJoints[joint1].TrackingState;

	// If we can't find either of these joints, exit 
	if ((joint0State == TrackingState_NotTracked) || (joint1State == TrackingState_NotTracked))
	{
		return;
	}

	// Don't draw if both points are inferred 推断
	if ((joint0State == TrackingState_Inferred) && (joint1State == TrackingState_Inferred))
	{
		return;
	}

	CvPoint p1 = cvPoint(depthSpacePosition[joint0].X, depthSpacePosition[joint0].Y),
			p2 = cvPoint(depthSpacePosition[joint1].X, depthSpacePosition[joint1].Y);

	// We assume all drawn bones are inferred unless BOTH joints are tracked
	if ((joint0State == TrackingState_Tracked) && (joint1State == TrackingState_Tracked))
	{
		//非常确定的骨架，用白色直线
		line(IndexImage, p1, p2, cvScalar(255, 255, 255));
	}
	else
	{
		//不确定的骨架，用红色直线
		line(IndexImage, p1, p2, cvScalar(0, 0, 255));
	}
}


// Constructor 构造器，括号中是变量的缺省值
CBodyBasics::CBodyBasics() :
m_pKinectSensor(NULL),
m_pCoordinateMapper(NULL),
m_pBodyFrameReader(NULL){}

// Destructor 析构器 释放对象
CBodyBasics::~CBodyBasics()
{
	SafeRelease(m_pBodyFrameReader);
	SafeRelease(m_pBodyIndexFrameReader);

	SafeRelease(m_pCoordinateMapper);

	if (m_pKinectSensor)
	{
		m_pKinectSensor->Close();
	}
	SafeRelease(m_pKinectSensor);
}


// 判断骨骼追踪情况：包括骨骼追踪完好且手部位置在肘上面  
bool GestureDetection(Joint & elbow, Joint & hand, ICoordinateMapper * myMapper)
{
	// 骨骼追踪完好
	if (elbow.TrackingState == TrackingState_Tracked && hand.TrackingState == TrackingState_Tracked)
	{
		DepthSpacePoint d_point;
		CvPoint p_e, p_h ;
		myMapper->MapCameraPointToDepthSpace(elbow.Position, &d_point);
		p_e.x = d_point.X;
		p_e.y = d_point.Y;
		myMapper->MapCameraPointToDepthSpace(hand.Position, &d_point);
		p_h.x = d_point.X;
		p_h.y = d_point.Y;
		//cout << "R_e:" << p_e.x << "," << p_e.y << endl;
		//cout << "R_h:" << p_h.x << "," << p_h.y << endl;
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
			if (hand_range >= 100 && elbow_range <= 50)
			{
				cout << "识别到挥手动作" << endl;
				return true;
			}
			else
			{
				cout << "未识别" << endl;
				return false;
			}
		}
		else
		{
			//cout << "不满足手掌高于手肘条件" << endl;
			return false;
		}
	}
	else return false;
}
