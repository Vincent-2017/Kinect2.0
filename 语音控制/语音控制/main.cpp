#include <Windows.h>
#include <atlstr.h>
#include <sphelper.h>
#include <sapi.h>
#include <comutil.h>
#include <string.h>

#pragma comment(lib,"sapi.lib")
#ifdef _UNICODE
#pragma   comment(lib,   "comsuppw.lib")  //_com_util::ConvertBSTRToString
#else
#pragma   comment(lib,   "comsupp.lib")  //_com_util::ConvertBSTRToString
#endif

#define GID_CMD_GR 333333
#define WM_RECOEVENT WM_USER+1

LRESULT CALLBACK WndProc(HWND, UINT, WPARAM, LPARAM);

TCHAR szAppName[] = TEXT("语音控制Demo");
BOOL b_initSR;
BOOL b_Cmd_Grammar;
CComPtr<ISpRecoContext> m_cpRecoCtxt;//语音识别程序接口
CComPtr<ISpRecoGrammar> m_cpCmdGramma;//识别语法
CComPtr<ISpRecognizer> m_cpRecoEngine; //语音识别引擎
int speak(wchar_t *str);

int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, PSTR szCmdLine, int iCmdShow)
{
	HWND hwnd;
	MSG msg;
	WNDCLASS wndclass;

	//窗口类结构体初始化值
	wndclass.cbClsExtra = 0;
	wndclass.cbWndExtra = 0;
	wndclass.hbrBackground = (HBRUSH)GetStockObject(WHITE_BRUSH);
	wndclass.hCursor = LoadCursor(NULL, IDC_ARROW);
	wndclass.hIcon = LoadIcon(NULL, IDI_APPLICATION);
	wndclass.hInstance = hInstance;
	wndclass.lpfnWndProc = WndProc;
	wndclass.lpszClassName = szAppName;
	wndclass.lpszMenuName = NULL;
	wndclass.style = CS_HREDRAW | CS_VREDRAW;

	//注册窗口类
	if (!RegisterClass(&wndclass))
	{
		//失败后提示并返回
		MessageBox(NULL, TEXT("This program requires Windows NT!"), szAppName, MB_ICONERROR);
		return 0;
	}

	//创建窗口
	hwnd = CreateWindow(szAppName,
		TEXT("语音识别"),
		WS_OVERLAPPEDWINDOW,
		CW_USEDEFAULT,
		CW_USEDEFAULT,
		CW_USEDEFAULT,
		CW_USEDEFAULT,
		NULL,
		NULL,
		hInstance,
		NULL);

	//显示窗口
	ShowWindow(hwnd, iCmdShow);
	UpdateWindow(hwnd);

	//进入消息循环
	while (GetMessage(&msg, NULL, 0, 0))
	{
		TranslateMessage(&msg);//翻译消息
		DispatchMessage(&msg);//分发消息
	}
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
	case WM_CREATE:
	{
					  //初始化COM端口
					  ::CoInitializeEx(NULL, COINIT_APARTMENTTHREADED);
					  //创建识别上下文接口
					  HRESULT hr = m_cpRecoEngine.CoCreateInstance(CLSID_SpSharedRecognizer);
					  if (SUCCEEDED(hr))
					  {
						  hr = m_cpRecoEngine->CreateRecoContext(&m_cpRecoCtxt);
					  }
					  else
					  {
						  MessageBox(hwnd, TEXT("引擎实例化出错"), TEXT("提示"), S_OK);
					  }
					  //设置识别消息，使计算机时刻监听语音消息
					  if (SUCCEEDED(hr))
					  {
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
					  b_Cmd_Grammar = TRUE;
					  if (FAILED(hr))
					  {
						  MessageBox(hwnd, TEXT("创建语法规则出错"), TEXT("提示"), S_OK);
					  }
					  hr = m_cpRecoCtxt->CreateGrammar(GID_CMD_GR, &m_cpCmdGramma);
					  hr = m_cpCmdGramma->LoadCmdFromFile(L"cmd.xml", SPLO_DYNAMIC);
					  if (FAILED(hr))
					  {
						  MessageBox(hwnd, TEXT("配置文件打开出错"), TEXT("提示"), S_OK);
					  }
					  b_initSR = TRUE;
					  //在开始识别时，激活语法进行识别
					  hr = m_cpCmdGramma->SetRuleState(NULL, NULL, SPRS_ACTIVE);
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
													  //取得消息结果
													  if (FAILED(event.RecoResult()->GetText(SP_GETWHOLEPHRASE, SP_GETWHOLEPHRASE, TRUE, &dstrText, NULL)))
													  {
														  dstrText = wszUnrecognized;
													  }
													  BSTR SRout;
													  dstrText.CopyToBSTR(&SRout);
													  char * lpszText2 = _com_util::ConvertBSTRToString(SRout);
													  if (b_Cmd_Grammar)
													  {
														  //MessageBoxA(0,lpszText2,"内容",0);
														  if (strcmp("腾讯QQ", lpszText2) == 0)
														  {
															  //MessageBox(0,TEXT("计算机"),TEXT("内容"),0);
															  speak(L"好的");
															  //打开QQ.exe
															  ShellExecuteA(NULL, "open", "D:\\QQ\\QQProtect\\Bin\\QQProtect.exe", 0, 0, 1);
														  }
														  if (strcmp("确定", lpszText2) == 0)
														  {
															  //按下回车键
															  keybd_event(VK_RETURN, 0, 0, 0);
															  keybd_event(VK_RETURN, 0, KEYEVENTF_KEYUP, 0);
														  }
														  if (strcmp("音乐", lpszText2) == 0)
														  {
															  speak(L"好的");
															  //调用系统程序wmplayer.exe播放音乐
															  ShellExecuteA(NULL, "open", "\"C:\\Program Files (x86)\\Windows Media Player\\wmplayer.exe\"", "C:\\Users\\KYT\\Desktop\\123.mp3", 0, 0);
														  }
													  }
							 }
							 }
						 }
						 break;
	}
	case WM_PAINT:
		hdc = BeginPaint(hwnd, &ps);
		EndPaint(hwnd, &ps);
		break;
	case WM_DESTROY:
		PostQuitMessage(0);
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
	::CoInitialize(NULL);
	//获得ISpVoice接口
	long hr = CoCreateInstance(CLSID_SpVoice, NULL, CLSCTX_ALL, IID_ISpVoice, (void **)&pVoice);
	hr = pVoice->Speak(str, 0, NULL);
	pVoice->Release();
	pVoice = NULL;
	//千万不要忘记
	::CoUninitialize();
	return TRUE;
}