#include "myKinect.h"
#include <iostream>
using namespace std;

int main()
{
	// 初始化
	CBodyBasics myKinect;
	HRESULT hr = myKinect.InitializeDefaultSensor();
	if (SUCCEEDED(hr)){
		while (1){
			// 更新数据
			myKinect.Update();
		}
	}
	else{
		cout << "kinect initialization failed!" << endl;
		system("pause");
	}
}