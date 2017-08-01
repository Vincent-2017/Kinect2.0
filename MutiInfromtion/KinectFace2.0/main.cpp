#include "global.h"

int data[10][2];
Point leftdepthpoint, rightdepthpoint, leftcolorpoint, rightcolorpoint;
int leftdepth, rightdepth;
string lhandstate, rhandstate;

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
	HRESULT hr = FALSE;
	/********************************   获得Kinect传感器  ********************************/
	IKinectSensor   * mySensor = nullptr;
	hr = GetDefaultKinectSensor(&mySensor);
	if (SUCCEEDED(hr))
		hr = mySensor->Open();

	/********************************   映射坐标系  ********************************/
	ICoordinateMapper   * myMapper = nullptr;
	if (SUCCEEDED(hr))
		hr = mySensor->get_CoordinateMapper(&myMapper);

	/********************************   彩色数据  ********************************/
	IColorFrameSource   * myColorSource = nullptr;
	if (SUCCEEDED(hr))
		hr = mySensor->get_ColorFrameSource(&myColorSource);
	IColorFrameReader   * myColorReader = nullptr;
	if (SUCCEEDED(hr))
		hr = myColorSource->OpenReader(&myColorReader);
	IFrameDescription   * mycolorDescription = nullptr;
	if (SUCCEEDED(hr))
		hr = myColorSource->get_FrameDescription(&mycolorDescription);
	int colorHeight = 0, colorWidth = 0;
	mycolorDescription->get_Height(&colorHeight);
	mycolorDescription->get_Width(&colorWidth);
	cout << "彩色图像大小：" << colorWidth << "*" << colorHeight << " 数据格式：CV_8UC4" << endl;
	IColorFrame * myColorFrame = nullptr;
	Mat color(colorHeight, colorWidth, CV_8UC4);
	Mat colormuti(colorHeight, colorWidth, CV_8UC4);

	/********************************   深度数据  ********************************/
	IDepthFrameSource   * myDepthSource = nullptr;   //取得深度数据
	if (SUCCEEDED(hr))
		hr = mySensor->get_DepthFrameSource(&myDepthSource);
	IDepthFrameReader   * myDepthReader = nullptr;
	if (SUCCEEDED(hr))
		hr = myDepthSource->OpenReader(&myDepthReader);    //打开深度数据的Reader
	IFrameDescription   * mydepthDescription = nullptr;
	if (SUCCEEDED(hr))
		hr = myDepthSource->get_FrameDescription(&mydepthDescription);
	int depthHeight = 0, depthWidth = 0;
	mydepthDescription->get_Height(&depthHeight);
	mydepthDescription->get_Width(&depthWidth);
	cout << "深度图像大小：" << depthWidth << " * " << depthHeight << " 数据格式：CV_16UC1" << endl;
	IDepthFrame * myDepthFrame = nullptr;
	Mat depth(depthHeight, depthWidth, CV_16UC1);    //建立图像矩阵
	Mat showdepth(depthHeight, depthWidth, CV_8UC1);
	Mat depthmuti(depthHeight, depthWidth, CV_8UC1);

	/********************************   红外数据  ********************************/
	IInfraredFrameSource * myInfraredSource = nullptr;
	if (SUCCEEDED(hr))
		hr = mySensor->get_InfraredFrameSource(&myInfraredSource);
	IInfraredFrameReader  * myInfraredReader = nullptr;
	if (SUCCEEDED(hr))
		hr = myInfraredSource->OpenReader(&myInfraredReader);
	IFrameDescription   * myinfraredDescription = nullptr;
	if (SUCCEEDED(hr))
		hr = myInfraredSource->get_FrameDescription(&myinfraredDescription);
	int infraredHeight = 0, infraredWidth = 0;
	myinfraredDescription->get_Height(&infraredHeight);
	myinfraredDescription->get_Width(&infraredWidth);
	cout << "红外图像大小：" << infraredWidth << " * " << infraredHeight << " 数据格式：CV_16UC1" << endl;
	IInfraredFrame  * myInfraredFrame = nullptr;
	Mat infrared(infraredHeight, infraredWidth, CV_16UC1);

	/********************************   人物索引  ********************************/
	IBodyIndexFrameSource* pBodyIndexFrameSource = NULL;//读取背景二值图
	if (SUCCEEDED(hr))
		hr = mySensor->get_BodyIndexFrameSource(&pBodyIndexFrameSource);
	IBodyIndexFrameReader*  m_pBodyIndexFrameReader;//用于背景二值图读取
	if (SUCCEEDED(hr))
		hr = pBodyIndexFrameSource->OpenReader(&m_pBodyIndexFrameReader);
	IFrameDescription   * mybodyindexDescription = nullptr;
	if (SUCCEEDED(hr))
		hr = pBodyIndexFrameSource->get_FrameDescription(&mybodyindexDescription);
	int bodyindexHeight = 0, bodyindexWidth = 0;
	mybodyindexDescription->get_Height(&bodyindexHeight);
	mybodyindexDescription->get_Width(&bodyindexWidth);
	cout << "人物图像大小：" << bodyindexWidth << " * " << bodyindexHeight << " 数据格式：CV_8UC3" << endl;
	IBodyIndexFrame  * myBodyIndexFrame = nullptr;
	Mat bodyindex(bodyindexHeight, bodyindexWidth, CV_8UC3);
	Mat bodyindexmuti(bodyindexHeight, bodyindexWidth, CV_8UC3);

	/********************************   骨骼数据  ********************************/
	IBodyFrameSource    * myBodySource = nullptr;
	if (SUCCEEDED(hr))
		mySensor->get_BodyFrameSource(&myBodySource);
	IBodyFrameReader    * myBodyReader = nullptr;
	if (SUCCEEDED(hr))
		myBodySource->OpenReader(&myBodyReader);
	cout << "骨骼图像没有可供显示的数据！" << endl;
	IBodyFrame  * myBodyFrame = nullptr;

	/********************************   面部数据  ********************************/
	IFaceFrameSource* facesource[BODY_COUNT];
	IFaceFrameReader* facereader[BODY_COUNT];
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
		hr = CreateFaceFrameSource(mySensor, 0, features, &facesource[i]);
		if (FAILED(hr))
		{
			std::cerr << "Error : CreateFaceFrameSource" << std::endl;
			return -1;
		}
		facesource[i]->OpenReader(&facereader[i]);
	}

	///********************************   socket通信  ********************************/
	////加载套接字
	//WSADATA wsaData;
	//char buff[1024];
	//memset(buff, 0, sizeof(buff));

	//if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0)
	//{
	//	printf("初始化Winsock失败");
	//	return 0;
	//}

	//SOCKADDR_IN addrSrv;
	//addrSrv.sin_family = AF_INET;
	//addrSrv.sin_port = htons(8080);//端口号
	//addrSrv.sin_addr.S_un.S_addr = inet_addr("192.168.137.1");//IP地址

	////创建套接字
	//SOCKET sockClient = socket(AF_INET, SOCK_STREAM, 0);
	//if (SOCKET_ERROR == sockClient){
	//	printf("Socket() error:%d", WSAGetLastError());
	//	return 0;
	//}

	////向服务器发出连接请求
	//if (connect(sockClient, (struct  sockaddr*)&addrSrv, sizeof(addrSrv)) == INVALID_SOCKET){
	//	printf("连接失败:%d", WSAGetLastError());
	//	return 0;
	//}
	//else
	//{
	//	//接收数据
	//	recv(sockClient, buff, sizeof(buff), 0);
	//	printf("%s\n", buff);
	//}

	/********************************   串口通信  ********************************/
	//1.打开指定串口
	HANDLE hComm = CreateFileA("COM3", // 串口名称(COMx)
		GENERIC_READ | GENERIC_WRITE, // 串口属性为可读/写
		0, // 串口设备必须被独占性的访问
		NULL, // 无安全属性
		OPEN_EXISTING, // 串口设备必须使用OPEN_EXISTING参数
		FILE_ATTRIBUTE_NORMAL, // 同步式 I/O
		0); // 对于串口设备而言此参数必须为0
	if (hComm == INVALID_HANDLE_VALUE)
	{
		//如果该串口不存在或者正被另外一个应用程序使用，
		//则打开失败，本程序退出
		return FALSE;
	}
	//2.设置串口参数：波特率、数据位、校验位、停止位等信息
	DCB dcb;
	GetCommState(hComm, &dcb); //获取该端口的默认参数
	//修改波特率
	dcb.BaudRate = 115200;
	//重新设置参数
	SetCommState(hComm, &dcb);

	while (1)
	{
		/********************************   彩色帧  ********************************/
		while (myColorReader->AcquireLatestFrame(&myColorFrame) != S_OK);
		myColorFrame->CopyConvertedFrameDataToArray(colorHeight * colorWidth * 4, color.data, ColorImageFormat_Bgra); // 图像大小、目标地址、图像格式
		colormuti = color.clone();
		myColorFrame->Release();
		//imshow("Color", color);

		/********************************   深度帧  ********************************/
		while (myDepthReader->AcquireLatestFrame(&myDepthFrame) != S_OK);
		myDepthFrame->CopyFrameDataToArray(depthHeight * depthWidth, (UINT16 *)depth.data); //先把数据存入16位的图像矩阵中
		myDepthFrame->Release();
		depth.convertTo(showdepth, CV_8UC1, 255.0 / 2500);   //再把16位转换为8位
		depthmuti = showdepth.clone();
		//imshow("Depth", showdepth);

		/********************************   红外帧  ********************************/
		while (myInfraredReader->AcquireLatestFrame(&myInfraredFrame) != S_OK);
		myInfraredFrame->CopyFrameDataToArray(infraredHeight * infraredWidth, (UINT16 *)infrared.data);
		myInfraredFrame->Release();
		//imshow("Infrared", infrared);

		/********************************   人物帧  ********************************/
		while (m_pBodyIndexFrameReader->AcquireLatestFrame(&myBodyIndexFrame) != S_OK);
		BYTE *bodyIndexArray = new BYTE[bodyindexHeight * bodyindexWidth];//背景二值图是8为uchar，有人是黑色，没人是白色
		myBodyIndexFrame->CopyFrameDataToArray(bodyindexHeight * bodyindexWidth, bodyIndexArray);
		myBodyIndexFrame->Release();
		uchar* bodyindexptr = (uchar*)bodyindex.data;
		for (int j = 0; j < bodyindexHeight * bodyindexWidth; ++j)
		{
			*bodyindexptr = bodyIndexArray[j]; ++bodyindexptr;
			*bodyindexptr = bodyIndexArray[j]; ++bodyindexptr;
			*bodyindexptr = bodyIndexArray[j]; ++bodyindexptr;
		}
		delete[] bodyIndexArray;
		bodyindexmuti = bodyindex.clone();
		//imshow("BodyIndex", bodyindex);

		/********************************   骨骼帧  ********************************/
		while (myBodyReader->AcquireLatestFrame(&myBodyFrame) != S_OK);
		IBody   **  myBodyArr = new IBody *[BODY_COUNT];
		for (int i = 0; i < BODY_COUNT; i++)	myBodyArr[i] = nullptr;
		if (myBodyFrame->GetAndRefreshBodyData(BODY_COUNT, myBodyArr) == S_OK)
#pragma omp parallel for 
		for (int i = 0; i < BODY_COUNT; i++)
		{
			BOOLEAN     result = false;
			if (myBodyArr[i]->get_IsTracked(&result) == S_OK && result)
			{
				Joint   myJointArr[JointType_Count];
				//如果侦测到就把关节数据输入到数组并画图
				if (myBodyArr[i]->GetJoints(JointType_Count, myJointArr) == S_OK)
				{
					DrawBody(colormuti, myJointArr, myMapper, USEColorSpace);
					drawhandstate(colormuti, myJointArr[JointType_HandLeft], myJointArr[JointType_HandRight], myBodyArr[i], myMapper, USEColorSpace);
					WaveGestureDetection(myJointArr[JointType_ElbowRight], myJointArr[JointType_HandRight], myMapper);
					leftcolorpoint = MapCameraPointToSomeSpace(myMapper, myJointArr[JointType_HandLeft], USEColorSpace);
					rightcolorpoint = MapCameraPointToSomeSpace(myMapper, myJointArr[JointType_HandRight], USEColorSpace);

					DrawBody(bodyindexmuti, myJointArr, myMapper, USEDepthSpace);
					drawhandstate(bodyindexmuti, myJointArr[JointType_HandLeft], myJointArr[JointType_HandRight], myBodyArr[i], myMapper, USEDepthSpace);
				
					DrawBody(depthmuti, myJointArr, myMapper, USEDepthSpace);
					drawhandstate(depthmuti, myJointArr[JointType_HandLeft], myJointArr[JointType_HandRight], myBodyArr[i], myMapper, USEDepthSpace);
					leftdepthpoint = MapCameraPointToSomeSpace(myMapper, myJointArr[JointType_HandLeft],  USEDepthSpace);
					rightdepthpoint = MapCameraPointToSomeSpace(myMapper, myJointArr[JointType_HandRight], USEDepthSpace);
					leftdepth = get_pixel(showdepth, leftdepthpoint);
					rightdepth = get_pixel(showdepth, rightdepthpoint);
					string leftstr , rightstr ;
					leftstr = lhandstate + "(" + to_string(int(leftcolorpoint.x)) + ", " + to_string(int(leftcolorpoint.y)) + ", " + to_string(leftdepth) + ")";
					rightstr = rhandstate + "(" + to_string(int(rightcolorpoint.x)) + ", " + to_string(int(rightcolorpoint.y)) + ", " + to_string(rightdepth) + ")";
					//if (leftcolorpoint.y - 100 > 1 && rightcolorpoint.y - 100 > 1 && leftcolorpoint.x - 200 > 1 && rightcolorpoint.x - 200 > 1)
					//{
					//	putText(colormuti, leftstr, Point(leftcolorpoint.x - 200, leftcolorpoint.y - 100), FONT_HERSHEY_SIMPLEX, 1.0f, Scalar(0, 0, 255, 255), 2, CV_AA);
					//	putText(colormuti, rightstr, Point(rightcolorpoint.x - 200, rightcolorpoint.y - 100), FONT_HERSHEY_SIMPLEX, 1.0f, Scalar(0, 0, 255, 255), 2, CV_AA);
					//	//cout << leftdepth << "  " << rightdepth << endl;
					//}
					if (lhandstate == "Close")
					{
						char buffs[100];
						strcpy(buffs, leftstr.c_str());
						//send(sockClient, buffs, sizeof(buffs), 0);
						//3.往串口写数据
						DWORD nNumberOfBytesToWrite = strlen(buffs); //将要写入的数据长度
						DWORD nBytesSent; //实际写入的数据长度
						WriteFile(hComm, buffs, nNumberOfBytesToWrite, &nBytesSent, NULL);
					}
					if (rhandstate == "Close")
					{
						char buffs[100];
						strcpy(buffs, rightstr.c_str());
						//send(sockClient, buffs, sizeof(buffs), 0);
						//3.往串口写数据
						DWORD nNumberOfBytesToWrite = strlen(buffs); //将要写入的数据长度
						DWORD nBytesSent; //实际写入的数据长度
						WriteFile(hComm, buffs, nNumberOfBytesToWrite, &nBytesSent, NULL);
					}
					////读串口数据
					//char lpReadBuf[1024] = { 0 }; //接收缓冲区长度为1024，内容都为0
					//DWORD nNumberOfBytesToRead = 1024; //最大读取1024个字节
					//DWORD nBytesRead;
					//ReadFile(hComm, lpReadBuf, nNumberOfBytesToRead, &nBytesRead, NULL);
					//if (strlen(lpReadBuf) != 0)
					//	printf("Read Data: %s \n", lpReadBuf);
				}
				// 把骨骼ID赋值给面部ID
				UINT64 trackingId = _UI64_MAX;
				hr = myBodyArr[i]->get_TrackingId(&trackingId);
				//cout << "追踪的骨骼ID: " << trackingId << endl;
				if (SUCCEEDED(hr))
				{
					facesource[i]->put_TrackingId(trackingId);
				}
			}
		}
		delete[]myBodyArr;
		myBodyFrame->Release();

		/********************************   面部帧  ********************************/
#pragma omp parallel for 
		for (int i = 0; i < BODY_COUNT; i++)
		{
			IFaceFrame*faceframe = nullptr;
			hr = facereader[i]->AcquireLatestFrame(&faceframe);
			if (faceframe == nullptr)
				continue;
			if (SUCCEEDED(hr) && faceframe != nullptr)
			{
				BOOLEAN tracked = false;
				hr = faceframe->get_IsTrackingIdValid(&tracked);
				if (SUCCEEDED(hr) && tracked)
				{
					IFaceFrameResult *faceresult = nullptr;
					UINT64 trackingID = NULL;
					faceframe->get_TrackingId(&trackingID);
					//cout << "追踪的人脸ID: " << trackingID << endl;
					hr = faceframe->get_FaceFrameResult(&faceresult);
					if (SUCCEEDED(hr))
					{
						PointF facepoint[FacePointType_Count];
						hr = faceresult->GetFacePointsInColorSpace(FacePointType_Count, facepoint);
						if (SUCCEEDED(hr))
						{
							circle(colormuti, cv::Point(facepoint[0].X, facepoint[0].Y), 5, Scalar(0, 0, 255, 255), -1, CV_AA); // Eye (Left)
							circle(colormuti, cv::Point(facepoint[1].X, facepoint[1].Y), 5, Scalar(0, 0, 255, 255), -1, CV_AA); // Eye (Right)
							circle(colormuti, cv::Point(facepoint[2].X, facepoint[2].Y), 5, Scalar(0, 0, 255, 255), -1, CV_AA); // Nose
							circle(colormuti, cv::Point(facepoint[3].X, facepoint[3].Y), 5, Scalar(0, 0, 255, 255), -1, CV_AA); // Mouth (Left)
							circle(colormuti, cv::Point(facepoint[4].X, facepoint[4].Y), 5, Scalar(0, 0, 255, 255), -1, CV_AA); // Mouth (Right)
						}
						RectI box;
						hr = faceresult->get_FaceBoundingBoxInColorSpace(&box);
						if (SUCCEEDED(hr))
						{
							cv::rectangle(colormuti, cv::Rect(box.Left, box.Top, box.Right - box.Left, box.Bottom - box.Top), Scalar(0, 0, 255, 255));
						}

						// 四元数 重点来了
						string result;
						Vector4 faceRotation;
						hr = faceresult->get_FaceRotationQuaternion(&faceRotation);
						if (SUCCEEDED(hr))
						{
							// 把四元数分解成三个方向的位置信息
							int pitch, yaw, roll;
							ExtractFaceRotationInDegrees(&faceRotation, &pitch, &yaw, &roll);
							UINT64 trackingId = _UI64_MAX;
							hr = faceframe->get_TrackingId(&trackingId);
							result += "FaceID: " + to_string(trackingId)
									+ "  Pitch, Yaw, Roll : " + to_string(pitch) + ", " + to_string(yaw) + ", " + to_string(roll)
									+"  Nose(" + to_string(int(facepoint[2].X)) + ", " + to_string(int(facepoint[2].Y)) + ")";
						}
						putText(colormuti, result, Point(0, 25 * (i + 1)), FONT_HERSHEY_SIMPLEX, 1.0f, Scalar(0, 0, 255, 255), 2, CV_AA);
					}
					SafeRelease(faceresult);
				}
			}
			SafeRelease(faceframe);
		}

		//Mat show;
		//cv::resize(colormuti, show, cv::Size(), 0.5, 0.5);
		cv::imshow("ColorMuti", colormuti);

		//imshow("BodyindexMuti", bodyindexmuti);

		//imshow("DepthMuti", depthmuti);

		if (cv::waitKey(34) == VK_ESCAPE)
		{
			break;
		}
	}

	for (int i = 0; i < BODY_COUNT; i++)
	{
		SafeRelease(facesource[i]);
		SafeRelease(facereader[i]);
	}

	myMapper->Release();

	mycolorDescription->Release();
	myColorReader->Release();
	myColorSource->Release();

	mydepthDescription->Release();
	myDepthReader->Release();
	myDepthSource->Release();

	myinfraredDescription->Release();
	myInfraredReader->Release();
	myInfraredSource->Release();

	mybodyindexDescription->Release();
	m_pBodyIndexFrameReader->Release();
	pBodyIndexFrameSource->Release();

	myBodyReader->Release();
	myBodySource->Release();

	mySensor->Close();
	mySensor->Release();

	cv::destroyAllWindows();
}

