#include "kinect.h"  
#include <iostream>  

using namespace std;
const int Number = 256;

int main()
{
	cout << "Hello, welcome to kinect world!" << endl;
	IKinectSensor* ikinectSensor;
	HRESULT hr = GetDefaultKinectSensor(&ikinectSensor);

	if (FAILED(hr)){
		cout << "No Kinect connect to you pc !" << endl;
		return 0;
	}
	cout << "Success connect to you pc !" << endl;

	BOOLEAN bIsOpen = 0; // detect kinect is open  
	BOOLEAN bAvaliable = 0; // detect kinect is avaliable  

	ikinectSensor->get_IsOpen(&bIsOpen);
	printf("bIsOpen is : %d \n", bIsOpen);

	if (!bIsOpen){
		hr = ikinectSensor->Open();
		if (FAILED(hr)){
			return 0;
		}

		// Because it take little time to open kinect, so just loop until device is avaliable.   
		while (!bAvaliable){
			ikinectSensor->get_IsAvailable(&bAvaliable);
		}
	}

	bIsOpen = 0;
	ikinectSensor->get_IsOpen(&bIsOpen);
	printf("bIsOpen is : %d \n", bIsOpen);

	printf("bAvaliable is : %d \n", bAvaliable);

	DWORD dwCapability = 0;
	ikinectSensor->get_KinectCapabilities(&dwCapability);
	printf("dwCapability is : %d \n", dwCapability);

	TCHAR build[Number] = { 0 };
	ikinectSensor->get_UniqueKinectId(Number, build);
	printf("UID is %d \n", build);

	ikinectSensor->Close();
	system("pause");
	return 0;
}