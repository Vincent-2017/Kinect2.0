#include <Windows.h>  
#include <Kinect.h>
#include <iostream>  
#include<ctime>  
#include<cassert>  
#include<process.h>  

using namespace std;

#define _DDDEBUG

static int CTRL = 0;

template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL){
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}

// 记录手势当前位置  
enum GesturePos{ // 
	NonePos = 0,
	Left,
	Right,
	Neutral // 中间
};

// 判断识别状态  
enum DetectionState{
	NoneState = 0,
	Success,
	Failed,
	InProgress // 识别过程中
};

// 判断手势需要的数据  
struct DataState{
	GesturePos Pos;     // 每个人的左右手位置  
	DetectionState State; // 识别状态
	int times; // 挥动次数
	time_t timestamp; // 时间戳
	void Reset() // 状态的重置
	{
		Pos = GesturePos::NonePos;
		State = DetectionState::NoneState;
		times = 0;
		timestamp = 0;
	}
};

// 完成手势判断逻辑功能  
class GestureDetection{
public:
	GestureDetection(float neutral_threshold, int times, double difftimes)
		: neutral_threshold(neutral_threshold)
		, times(times)
		, difftimes(difftimes)
		, left_hand(0)
		, right_hand(1) // 缺省值 right_hand = 1
	{
		wave_datas[0][left_hand].Reset();
		wave_datas[0][right_hand].Reset();
	}
	// 功能：循环接收骨骼数据，如果识别出为挥手动作则输出：success，  
	// 识别失败输出：failed，  
	void Update(IBody * frame)
	{

		if (frame == NULL)
			return;
		JudgeState(frame, wave_datas[1][right_hand], true);
	}
private:
	DataState wave_datas[1][2];        // 记录每个人，每只手的状态  
	const int left_hand;                            // 左手 ID  
	const int right_hand;                           // 右手 ID  
	// 中间位置阀值：在该范围内的都认为手在中间位置（相对于肘部的 x 坐标）  
	const float neutral_threshold;
	// 挥手次数阀值，达到该次数认为是挥手  
	const int times;
	// 时间限制，如果超过该时间差依然识别不出挥手动作则认为识别失败  
	const double difftimes;
	// 胳膊和手
	int elbow ;
	int hand;

	// 判断当前的状态成功输出：success，并生成事件：DetectionEvent   
	// 失败输出 failed，供 UpDate 函数调用  
	void JudgeState(IBody *n_body, DataState& data, int handID )
	{

		Joint joints[JointType_Count]; // 定义骨骼信息

		n_body->GetJoints(JointType::JointType_Count, joints);  // 获取骨骼信息节点

		if (handID == 0)
		{
			elbow = JointType_ElbowLeft;
			hand = JointType_HandLeft;
		}
		else
		{
			elbow = JointType_ElbowRight;
			hand = JointType_HandRight;
		}

		if (!IsSkeletonTrackedWell(n_body, handID))  //  如果手部的位置在肘部之上  则认为为真
		{
			if (data.State == InProgress)
			{
#ifdef _DDEBUG
				cout << "not a well skeleton, detection failed!\n";
#endif  
				data.Reset();
				return;
			}
		}

		float curpos = joints[hand].Position.X;
		float center = joints[elbow].Position.X;  //  得到人手部和肘部的X坐标的位置  都是右手

		if (!IsNeutral(curpos, center))  //  如果手部不是在中立的位置
		{
			if (data.Pos == NonePos)
			{
#ifdef _DDEBUG  
				cout << "found!\n";
#endif  

				data.times++;


				if (get_length(curpos, center) == -1)
				{
					data.Pos = Left;
				}
				else if (get_length(curpos, center) == 1)
				{
					data.Pos = Right;
				}
				cout << "times:" << data.times << endl;
				if (data.Pos == Left)
				{
					cout << "left !\n";
				}
				else if (data.Pos == Right)
				{
					cout << "right!\n";
				}
				else
					cout << "you can't see me!\n";

				data.State = InProgress;
				data.timestamp = time(NULL);

			}

			else if (((data.Pos == Left) && get_length(curpos, center) == 1) || ((data.Pos == Right) && get_length(curpos, center) == -1))  // 左摆找右摆  右摆找左摆
			{

				assert(data.State == InProgress);
				data.times++;
				data.Pos = (data.Pos == Left) ? Right : Left;
#ifdef _DDDEBUG  
				cout << "times:" << data.times << endl;
				if (data.Pos == Left)
				{
					cout << "left !\n";
				}
				else if (data.Pos == Right)
				{
					cout << "right!\n";
				}
				else
					cout << "you can't see me!\n";
#endif  
				if (data.times >= times)
				{
#ifdef _DDDEBUG  
					cout << "success!\n";
#endif  
					CTRL = 1;
					data.Reset();
				}
				else if (difftime(time(NULL), data.timestamp) > difftimes)
				{
#ifdef _DDDEBUG  
					cout << "time out, detection failed!\n";
					cout << "data.times : " << data.times << endl;
#endif  
					data.Reset();
				}
			}
		}

	}

	bool IsLeftSide(float curpos, float center)
	{
		int i = 0;
		i = get_length(curpos, center);
		if (i == -1)
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	bool IsRightSide(float curpos, float center)
	{
		int i = 0;
		i = get_length(curpos, center);
		if (i == 1)
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	bool IsNeutral(float curpos, float center) // 参数分别为手部的位置和肘部的位置  判断是否是中立的状态
	{
		int i = 0;
		i = get_length(curpos, center);
		if (i == 0)
		{
			return true;  // 是中立的状态
		}
		else
		{
			return false;
		}
	}

	int get_length(float shou, float zhou)   //  输入：手和肘的X坐标  输出：右边是 1 左边是 -1 中间是 0
	{
		if (shou >= 0 && zhou >= 0) 
		{
			if ((shou - zhou) > neutral_threshold)
			{
				return 1; // 在右边
			}
			else if ((shou - zhou) < neutral_threshold || (zhou - shou) > -neutral_threshold)
			{
				return 0;  // 中立
			}
			else
			{
				return -1;   // 左边
			}
		}
		else if (shou >= 0 && zhou <= 0)
		{
			if ((shou + (-zhou)) > neutral_threshold)
			{
				return 1; // 右边
			}
			else
			{
				return 0; // 中立
			}
		}
		else if (shou <= 0 && zhou >= 0)
		{
			if (((-shou) + zhou) > neutral_threshold)
			{
				return -1; // 左边
			}
			else
			{
				return 0; // 中立
			}
		}
		else
		{
			if ((-shou) >= (-zhou))
			{
				if (((-shou) + zhou) > neutral_threshold)
				{
					return -1; // 左边
				}
				else if (((-shou) + zhou) < neutral_threshold)
				{
					return 0; // 中立
				}
				else
				{
					return 1;  // 右边
				}
			}
			else
			{
				if (((-zhou) + shou) > neutral_threshold)
				{
					return 1; // 右边
				}
				else
				{
					return 0; // 中立
				}
			}
		}

	}

	// 判断骨骼追踪情况：包括骨骼追踪完好且手部位置在肘上面  
	bool IsSkeletonTrackedWell(IBody * n_body, int handID )
	{
		Joint joints[JointType_Count];
		n_body->GetJoints(JointType::JointType_Count, joints);
		if (handID == 0)
		{
			elbow = JointType_ElbowLeft;
			hand = JointType_HandLeft;
		}
		else
		{
			elbow = JointType_ElbowRight;
			hand = JointType_HandRight;
		}
		if (joints[hand].Position.Y > joints[elbow].Position.Y) return true;
		else return false;
	}
};

int main()
{
	IKinectSensor *kinect = NULL;
	HRESULT hr = S_OK;
	hr = GetDefaultKinectSensor(&kinect);  //  得到默认的设备

	if (FAILED(hr) || kinect == NULL)
	{
		cout << "创建 sensor 失败\n";
		return -1;
	}
	if (kinect->Open() != S_OK) // 是否打开成功
	{
		cout << "Kinect sensor 没准备好\n";
		return -1;
	}

	IBodyFrameSource *bady = nullptr;  // 获取源
	hr = kinect->get_BodyFrameSource(&bady);

	IBodyFrameReader *pBodyReader;

	hr = bady->OpenReader(&pBodyReader); // 打开获取骨骼信息的  Reader
	if (FAILED(hr)){
		std::cerr << "Error : IBodyFrameSource::OpenReader()" << std::endl;
		return -1;
	}

	cout << "开始检测\n";

	GestureDetection gesture_detection(0.05, 5, 5);

	while (1)
	{
		IBodyFrame* pBodyFrame = nullptr;
		hr = pBodyReader->AcquireLatestFrame(&pBodyFrame);

		if (SUCCEEDED(hr)){
			IBody* pBody[BODY_COUNT] = { 0 }; // 默认的是 6 个骨骼 ，初始化所有的骨骼信息
			//更新骨骼数据  
			hr = pBodyFrame->GetAndRefreshBodyData(BODY_COUNT, pBody); // 刷新骨骼信息（6个）
			if (SUCCEEDED(hr))
			{
				BOOLEAN bTracked = false;

				for (int i = 0; i < 6; i++)
				{
					hr = pBody[i]->get_IsTracked(&bTracked); // 检查是否被追踪

					if (SUCCEEDED(hr) && bTracked)
					{

						gesture_detection.Update(pBody[i]);
					}
				}
			}
			for (int count = 0; count < BODY_COUNT; count++){
				SafeRelease(pBody[count]);
			}
		}
		if (CTRL == 1)
		{
			break;  //  识别成功以后  跳出识别程序
		}

		SafeRelease(pBodyFrame);  // 别忘了释放
	}

	kinect->Close();  // 关闭设备
	system("pause");
	return 0;
}
