#include <Windows.h>
#include <atlstr.h>
#include <sphelper.h>
#include <sapi.h>
#include <comutil.h>
#include <string.h>
#include <gdiplus.h>

using namespace std;

using namespace Gdiplus;
ULONG_PTR m_gdiplusToken;

#pragma comment(lib,"sapi.lib")
#pragma comment(lib,"gdiplus.lib")
#ifdef _UNICODE
#pragma   comment(lib,   "comsuppw.lib")  //_com_util::ConvertBSTRToString
#else
#pragma   comment(lib,   "comsupp.lib")  //_com_util::ConvertBSTRToString
#endif

#define GID_CMD_GR 333333
#define WM_RECOEVENT WM_USER+1

// 必须要进行前导声明
LRESULT CALLBACK WndProc(HWND, UINT, WPARAM, LPARAM);

HANDLE hComm;

BOOL b_initSR;
BOOL b_Cmd_Grammar;
CComPtr<ISpRecognizer> m_cpRecoEngine; // 语音识别引擎(recognition)的接口
CComPtr<ISpRecoContext> m_cpRecoCtxt; // 识别引擎上下文(context)的接口
CComPtr<ISpRecoGrammar> m_cpCmdGramma; //识别语法(grammer)接口

int speak(wchar_t *str);
void sendcode(HANDLE Com, char* str);

int WINAPI WinMain(
	HINSTANCE hInstance,     // 当前应用程序的实例句柄
	HINSTANCE hPrevInstance, // 前一个实例
	PSTR szCmdLine,			 // 命令行参数 字符串 char*
	int iCmdShow)			 // 主窗口的显示方式
{
	// 初始化GDI+    
	Gdiplus::GdiplusStartupInput gdiplusStartupInput;
	Gdiplus::GdiplusStartup(&m_gdiplusToken, &gdiplusStartupInput, NULL);

	/********************************   串口通信  ********************************/
	//1.打开指定串口
	hComm = CreateFileA("COM3", // 串口名称(COMx)
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


	HWND hwnd;
	MSG msg;
	// 类名
	TCHAR szAppName[] = TEXT("语音控制");
	// 设计窗口类
	WNDCLASS wndclass;

	//窗口类结构体初始化值
	wndclass.cbClsExtra = 0; // cbClsExtra和cbWndExtra通常不需要，设为0就OK
	wndclass.cbWndExtra = 0;
	wndclass.hbrBackground = (HBRUSH)GetStockObject(WHITE_BRUSH);
	wndclass.hCursor = LoadCursor(NULL, IDC_ARROW);
	wndclass.hIcon = LoadIcon(NULL, IDI_APPLICATION);
	wndclass.hInstance = hInstance;
	wndclass.lpfnWndProc = WndProc; // 设置回调函数名称,设置你用哪个WindowProc来处理消息
	wndclass.lpszClassName = szAppName;
	wndclass.lpszMenuName = NULL;
	wndclass.style = CS_HREDRAW | CS_VREDRAW; // 窗口类的特征,同时具备水平重画和垂直重画

	//注册窗口类
	if (!RegisterClass(&wndclass))
	{
		//失败后提示并返回
		MessageBox(NULL, TEXT("This program requires Windows NT!"), szAppName, MB_ICONERROR);
		return 0;
	}

	//创建窗口 获得窗口类的句柄
	hwnd = CreateWindow(
		szAppName,				// 类名，要和刚才注册的一致
		TEXT("语音识别"),		// 窗口标题文字  
		WS_OVERLAPPEDWINDOW,	// 窗口外观样式
		CW_USEDEFAULT,			// 窗口相对于父级的X坐标  
		CW_USEDEFAULT,			// 窗口相对于父级的Y坐标
		CW_USEDEFAULT,			// 窗口的宽度 
		CW_USEDEFAULT,			// 窗口的高度 
		NULL,					// 没有父窗口，为NULL 
		NULL,					// 没有菜单，为NULL 
		hInstance,				// 当前应用程序的实例句柄 
		NULL);					// 没有附加数据，为NULL 
	if (hwnd == NULL)           // 检查窗口是否创建成功  
		return 0;
	// 显示窗口
	ShowWindow(hwnd, iCmdShow);
	// 更新窗口 
	UpdateWindow(hwnd);

	// 进入消息循环
	while (GetMessage(
		&msg, // 指向MSG结构的指针
		NULL, // 句柄，通常用NULL，因为会捕捉整个应用程序的消息
		0,	  // 后两个过滤消息的，指定哪个范围内的消息我接收，在此范围之外的消息我拒收，如果不过滤就全设为0
		0))
	{
		TranslateMessage(&msg);	// 翻译消息
		DispatchMessage(&msg);	// 分发消息
	}
	Gdiplus::GdiplusShutdown(m_gdiplusToken);
	return msg.wParam;
}

/*
*消息回调函数,由操作系统调用
*/
LRESULT CALLBACK WndProc(HWND hwnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	HDC hdc;
	PAINTSTRUCT ps;

	switch (message)
	{
	case WM_CREATE: // 窗口创建
	{
					  // 初始化COM端口
					  if (FAILED(::CoInitialize(NULL)))
						  MessageBox(NULL, (LPCWSTR)L"COM接口初始化失败！", (LPCWSTR)L"提示", MB_ICONWARNING | MB_CANCELTRYCONTINUE | MB_DEFBUTTON2);

					  // 创建Share型识别引擎
					  HRESULT hr = m_cpRecoEngine.CoCreateInstance(CLSID_SpSharedRecognizer);
					  if (SUCCEEDED(hr))
					  {
						  // 创建识别上下文接口
						  hr = m_cpRecoEngine->CreateRecoContext(&m_cpRecoCtxt);
					  }
					  else
					  {
						  MessageBox(hwnd, TEXT("引擎实例化出错"), TEXT("提示"), S_OK);
					  }
					  if (SUCCEEDED(hr))
					  {
						  // 设置识别消息，使计算机时刻监听语音消息
						  hr = m_cpRecoCtxt->SetNotifyWindowMessage(hwnd, WM_RECOEVENT, 0, 0);
					  }
					  else
					  {
						  MessageBox(hwnd, TEXT("创建上下文接口出错"), TEXT("提示"), S_OK);
					  }
					  //设置我们感兴趣的事件
					  if (SUCCEEDED(hr))
					  {
						  ULONGLONG ullMyEvents = SPFEI(SPEI_SOUND_START) | SPFEI(SPEI_RECOGNITION) | SPFEI(SPEI_SOUND_END);
						  hr = m_cpRecoCtxt->SetInterest(ullMyEvents, ullMyEvents);
					  }
					  else
					  {
						  MessageBox(hwnd, TEXT("设置识别消息出错"), TEXT("提示"), S_OK);
					  }

					  //创建语法规则
					  //dictation听说式
					  //hr = m_cpRecoCtxt->CreateGrammar(GIDDICTATION, &m_cpDictationGrammar);
					  //if (SUCCEEDED(hr))
					  //{
					  //  hr = m_cpDictationGrammar->LoadDictation(NULL, SPLO_STATIC);//加载词典
					  //}

					  //创建语法规则
					  //C&C命令式，此时语法文件使用xml格式
					  b_Cmd_Grammar = TRUE;
					  hr = m_cpRecoCtxt->CreateGrammar(GID_CMD_GR, &m_cpCmdGramma);
					  if (FAILED(hr))
					  {
						  MessageBox(hwnd, TEXT("创建语法规则出错"), TEXT("提示"), S_OK);
					  }
					  hr = m_cpCmdGramma->LoadCmdFromFile(L"cmd.xml", SPLO_DYNAMIC);
					  if (FAILED(hr))
					  {
						  MessageBox(hwnd, TEXT("配置文件打开出错"), TEXT("提示"), S_OK);
					  }
					  b_initSR = TRUE;
					  //在开始识别时，激活语法进行识别
					  // hr = m_cpDictationGrammar->SetDictationState(SPRS_ACTIVE); // dictation
					  hr = m_cpCmdGramma->SetRuleState(NULL, NULL, SPRS_ACTIVE); // C&C

					  speak(L"您需要帮助吗？ 请说出“开始聆听”来激活语音控制");

					  break;
	}
	case WM_RECOEVENT:
	{
						 RECT rect;
						 GetClientRect(hwnd, &rect);
						 hdc = GetDC(hwnd);
						 USES_CONVERSION;
						 CSpEvent event;
						 while (event.GetFrom(m_cpRecoCtxt) == S_OK)
						 {
							 switch (event.eEventId)
							 {
							 case SPEI_RECOGNITION:
							 {
													  static const WCHAR wszUnrecognized[] = L"<Unrecognized>";
													  CSpDynamicString dstrText;
													  // 取得消息结果
													  if (FAILED(event.RecoResult()->GetText(SP_GETWHOLEPHRASE, SP_GETWHOLEPHRASE, TRUE, &dstrText, NULL)))
													  {
														  dstrText = wszUnrecognized;
													  }
													  BSTR SRout;
													  dstrText.CopyToBSTR(&SRout);
													  char * lpszText2 = _com_util::ConvertBSTRToString(SRout);
													  if (b_Cmd_Grammar)
													  {
														  if (strcmp("带路", lpszText2) == 0)
														  {
															  speak(L"好的，请说出目的地");
															  sendcode(hComm, "show the way \n");
														  }
														  if (strcmp("前台", lpszText2) == 0)
														  {
															  speak(L"好的，请跟我到前台");
															  sendcode(hComm, "reception desk \n");
														  }
														  if (strcmp("洗手间", lpszText2) == 0)
														  {
															  speak(L"好的，请跟我来洗手间");
															  sendcode(hComm, "bathroom \n");
														  }
														  if (strcmp("候客厅", lpszText2) == 0)
														  {
															  speak(L"好的，请跟我来候客厅");
															  sendcode(hComm, "Waiting room \n");
														  }

														  if (strcmp("饮食", lpszText2) == 0)
														  {
															  speak(L"好的，请说出您的需求");
															  sendcode(hComm, "food and drink \n");
														  }
														  if (strcmp("可乐", lpszText2) == 0)
														  {
															  speak(L"好的，即将为您提供可乐");
															  sendcode(hComm, "cola \n");
														  }
														  if (strcmp("咖啡", lpszText2) == 0)
														  {
															  speak(L"好的，即将为您提供咖啡");
															  sendcode(hComm, "coffee \n");
														  }
														  if (strcmp("盖浇饭", lpszText2) == 0)
														  {
															  speak(L"好的，即将为您准备盖浇饭，请稍等");
															  sendcode(hComm, "Rice \n");
														  }


														  if (strcmp("应用", lpszText2) == 0)
														  {
															  speak(L"好的，请说出您想打开的应用");
															  sendcode(hComm, "application \n");
														  }
														  if (strcmp("腾讯", lpszText2) == 0)
														  {
															  speak(L"好的，即将为您打开QQ");
															  ShellExecuteA(NULL, "open", "C:\\SoftWare\\TIM\\Bin\\TIM.exe", 0, 0, 1);
															  sendcode(hComm, "QQ \n");
														  }
														  if (strcmp("浏览器", lpszText2) == 0)
														  {
															  speak(L"好的，即将为您打开QQ浏览器");
															  ShellExecuteA(NULL, "open", "C:\\SoftWare\\QQ浏览器\\QQBrowser.exe", 0, 0, 1);
															  sendcode(hComm, "QQ Browser \n");
														  }
														  if (strcmp("音乐", lpszText2) == 0)
														  {
															  speak(L"好的，即将为您打开音乐播放器");
															  ShellExecuteA(NULL, "open", "‪C:\\SoftWare\\CloudMusic\\cloudmusic.exe", 0, 0, 1);
															  sendcode(hComm, "Music \n");
														  }

														  if (strcmp("自动跟随", lpszText2) == 0)
														  {
															  speak(L"好的，已打开自动跟随服务");
															  ShellExecuteA(NULL, "open", "C:\\Users\\VincentZHOU\\Desktop\\kinect\\MutiInformtion.exe", 0, 0, 1);
															  sendcode(hComm, "automatic following \n");
														  }
													  }
							 }
							 }
						 }
						 break;
	}
	case WM_PAINT:
	{
					 int width, height;
					 hdc = BeginPaint(hwnd, &ps);
					 // TODO:  在此添加任意绘图代码...  
					 //加载图像  
					 Image image(L"1.png");
					 if (image.GetLastStatus() != Status::Ok){
						 MessageBox(hwnd, L"加载图片失败!", L"提示", MB_OK);
						 return -1;
					 }
					 //取得宽度和高度  
					 width = image.GetWidth();
					 height = image.GetHeight();
					 //绘图  
					 Graphics graphics(hdc);
					 graphics.DrawImage(&image, 0, 0, width, height);
					 EndPaint(hwnd, &ps);
					 break;
	}

	case WM_DESTROY:
		PostQuitMessage(0); // 窗口销毁
		break;
	}
	return DefWindowProc(hwnd, message, wParam, lParam);
}

#pragma comment(lib,"ole32.lib")//CoInitialize CoCreateInstance 需要调用ole32.dll

/*
*语音合成函数，朗读字符串str
*/
int speak(wchar_t *str)
{
	ISpVoice * pVoice = NULL;
	// 初始化com接口
	if (FAILED(::CoInitialize(NULL)))
		MessageBox(NULL, (LPCWSTR)L"COM接口初始化失败！", (LPCWSTR)L"提示", MB_ICONWARNING | MB_CANCELTRYCONTINUE | MB_DEFBUTTON2);
	//获得ISpVoice接口
	long hr = CoCreateInstance(CLSID_SpVoice, NULL, CLSCTX_ALL, IID_ISpVoice, (void **)&pVoice);
	if (SUCCEEDED(hr))
	{
		pVoice->SetVolume((USHORT)100); //设置音量，范围是 0 -100
		pVoice->SetRate(0); //设置速度，范围是 -10 - 10
		hr = pVoice->Speak(str, 0, NULL);
		pVoice->Release();
		pVoice = NULL;
	}
	//千万不要忘记 释放com资源
	::CoUninitialize();
	return TRUE;
}

void sendcode(HANDLE Com , char* str)
{
	char buffs[100];
	strcpy(buffs, str);
	//send(sockClient, buffs, sizeof(buffs), 0);
	//3.往串口写数据
	DWORD nNumberOfBytesToWrite = strlen(buffs); //将要写入的数据长度
	DWORD nBytesSent; //实际写入的数据长度
	WriteFile(Com, buffs, nNumberOfBytesToWrite, &nBytesSent, NULL);
}