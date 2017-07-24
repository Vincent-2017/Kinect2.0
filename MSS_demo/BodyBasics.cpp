//------------------------------------------------------------------------------
// <copyright file="BodyBasics.cpp" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

#include "stdafx.h"
#include <strsafe.h>
#include "resource.h"
#include "BodyBasics.h"


using namespace std;



/// <summary>
/// Entry point for the application
/// </summary>
/// <param name="hInstance">handle to the application instance</param>
/// <param name="hPrevInstance">always 0</param>
/// <param name="lpCmdLine">command line arguments</param>
/// <param name="nCmdShow">whether to display minimized, maximized, or normally</param>
/// <returns>status</returns>
int APIENTRY wWinMain(    
	_In_ HINSTANCE hInstance,
    _In_opt_ HINSTANCE hPrevInstance,
    _In_ LPWSTR lpCmdLine,
    _In_ int nShowCmd
)
{
    UNREFERENCED_PARAMETER(hPrevInstance);
    UNREFERENCED_PARAMETER(lpCmdLine);

    CBodyBasics application;
    application.Run(hInstance, nShowCmd);
}

/// <summary>
/// Constructor
/// </summary>
CBodyBasics::CBodyBasics() 
{
    
}
  

/// <summary>
/// Destructor
/// </summary>
CBodyBasics::~CBodyBasics()
{
	  
	m_cpRecoEngine.Release();//释放语音引擎
	m_cpRecoCtxt.Release();
	m_cpCmdGrammar.Release();
	m_cpAudio.Release();
}

/// <summary>
/// Creates the main window and begins processing
/// </summary>
/// <param name="hInstance">handle to the application instance</param>
/// <param name="nCmdShow">whether to display minimized, maximized, or normally</param>
int CBodyBasics::Run(HINSTANCE hInstance, int nCmdShow)
{
    MSG       msg = {0};
    WNDCLASS  wc;

    // Dialog custom window class
    ZeroMemory(&wc, sizeof(wc));
    wc.style         = CS_HREDRAW | CS_VREDRAW;
    wc.cbWndExtra    = DLGWINDOWEXTRA;
    wc.hCursor       = LoadCursorW(NULL, IDC_ARROW);
    wc.hIcon         = LoadIconW(hInstance, MAKEINTRESOURCE(IDI_APP));
    wc.lpfnWndProc   = DefDlgProcW;
    wc.lpszClassName = L"BodyBasicsAppDlgWndClass";

    if (!RegisterClassW(&wc))
    {
        return 0;
    }

    // Create main application window
    HWND hWndApp = CreateDialogParamW(
        NULL,
        MAKEINTRESOURCE(IDD_APP),
        NULL,
        (DLGPROC)CBodyBasics::MessageRouter, 
        reinterpret_cast<LPARAM>(this));

    // Show window
    ShowWindow(hWndApp, nCmdShow);

    // Main message loop
    while (WM_QUIT != msg.message)
    {
        

        while (PeekMessageW(&msg, NULL, 0, 0, PM_REMOVE))
        {
            // If a dialog message will be taken care of by the dialog proc
            if (hWndApp && IsDialogMessageW(hWndApp, &msg))
            {
                continue;
            }

            TranslateMessage(&msg);
            DispatchMessageW(&msg);
        }
    }

    return static_cast<int>(msg.wParam);
}


/// <summary>
/// Handles window messages, passes most to the class instance to handle
/// </summary>
/// <param name="hWnd">window message is for</param>
/// <param name="uMsg">message</param>
/// <param name="wParam">message data</param>
/// <param name="lParam">additional message data</param>
/// <returns>result of message processing</returns>
LRESULT CALLBACK CBodyBasics::MessageRouter(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
    CBodyBasics* pThis = NULL;
    
    if (WM_INITDIALOG == uMsg)
    {
        pThis = reinterpret_cast<CBodyBasics*>(lParam);
        SetWindowLongPtr(hWnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(pThis));
    }
    else
    {
        pThis = reinterpret_cast<CBodyBasics*>(::GetWindowLongPtr(hWnd, GWLP_USERDATA));
    }

    if (pThis)
    {
        return pThis->DlgProc(hWnd, uMsg, wParam, lParam);
    }

    return 0;
}

/// <summary>
/// Handle windows messages for the class instance
/// </summary>
/// <param name="hWnd">window message is for</param>
/// <param name="uMsg">message</param>
/// <param name="wParam">message data</param>
/// <param name="lParam">additional message data</param>
/// <returns>result of message processing</returns>
LRESULT CALLBACK CBodyBasics::DlgProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
    UNREFERENCED_PARAMETER(wParam);
    UNREFERENCED_PARAMETER(lParam);

    switch (message)
    {
        case WM_INITDIALOG:
        {
            // Bind application window handle
            m_hWnd = hWnd;

            // Init MSS
			MSSListen();
			MSSSpeak(LPCTSTR(_T("您需要帮助吗 ")));
			

         
        }
        break;

        // If the titlebar X is clicked, destroy app
        case WM_CLOSE:
            DestroyWindow(hWnd);
            break;

        case WM_DESTROY:
            // Quit the main message pump
            PostQuitMessage(0);
            break;

    }

	USES_CONVERSION;
	CSpEvent event;

	if (m_cpRecoCtxt)
	{
		while (event.GetFrom(m_cpRecoCtxt) == S_OK){

			switch (event.eEventId)
			{
			case SPEI_RECOGNITION:
			{
									 //识别出了语音
									 m_bGotReco = TRUE; 

									 static const WCHAR wszUnrecognized[] = L"<Unrecognized>";

									 CSpDynamicString dstrText;

									 ////取得识别结果 
									 if (FAILED(event.RecoResult()->GetText(SP_GETWHOLEPHRASE, SP_GETWHOLEPHRASE, TRUE, &dstrText, NULL)))
									 {
										 dstrText = wszUnrecognized;
									 }

									 BSTR SRout;
									 dstrText.CopyToBSTR(&SRout);
									 CString Recstring;
									 Recstring.Empty();
									 Recstring = SRout;

									
									 if (Recstring == "发短信")
									 {
										 //MessageBox(NULL, (LPCWSTR)L"好的", (LPCWSTR)L"提示", MB_OK);
										 MSSSpeak(LPCTSTR(_T("好，马上发短信！")));
										
									 }

									 else if (Recstring == "李雷")
									 {
										 MSSSpeak(LPCTSTR(_T("好久没看见他了，真是 long time no see")));
									 }

									
										 
									
			}
				break;
			}
		}
	}

    return FALSE;
}



//文字转语音
void  CBodyBasics::MSSSpeak(LPCTSTR speakContent)
{
	ISpVoice *pVoice = NULL;

	//初始化COM接口

	if (FAILED(::CoInitialize(NULL)))
		MessageBox(NULL, (LPCWSTR)L"COM接口初始化失败！", (LPCWSTR)L"提示", MB_ICONWARNING | MB_CANCELTRYCONTINUE | MB_DEFBUTTON2);
		
	//获取SpVoice接口

	HRESULT hr = CoCreateInstance(CLSID_SpVoice, NULL, CLSCTX_ALL, IID_ISpVoice, (void**)&pVoice);


	if (SUCCEEDED(hr))
	{
		pVoice->SetVolume((USHORT)100); //设置音量，范围是 0 -100
		pVoice->SetRate(2); //设置速度，范围是 -10 - 10
		hr = pVoice->Speak(speakContent, 0, NULL);

		pVoice->Release();

		pVoice = NULL;
	}

	//释放com资源
	::CoUninitialize();
}


//语音转文字
void  CBodyBasics::MSSListen()
{

	//初始化COM接口

	if (FAILED(::CoInitialize(NULL)))
		MessageBox(NULL, (LPCWSTR)L"COM接口初始化失败！", (LPCWSTR)L"提示", MB_ICONWARNING | MB_CANCELTRYCONTINUE | MB_DEFBUTTON2);

	
	HRESULT hr = m_cpRecoEngine.CoCreateInstance(CLSID_SpSharedRecognizer);//创建Share型识别引擎
	if (SUCCEEDED(hr))
	{
		

		hr = m_cpRecoEngine->CreateRecoContext(&m_cpRecoCtxt);//创建识别上下文接口

		hr = m_cpRecoCtxt->SetNotifyWindowMessage(m_hWnd, WM_RECORD, 0, 0);//设置识别消息

		const ULONGLONG ullInterest = SPFEI(SPEI_SOUND_START) | SPFEI(SPEI_SOUND_END) | SPFEI(SPEI_RECOGNITION);//设置我们感兴趣的事件
		hr = m_cpRecoCtxt->SetInterest(ullInterest, ullInterest);

		hr = SpCreateDefaultObjectFromCategoryId(SPCAT_AUDIOIN, &m_cpAudio);
		m_cpRecoEngine->SetInput(m_cpAudio, true);
		


		//创建语法规则
		//dictation听说式
		//hr = m_cpRecoCtxt->CreateGrammar(GIDDICTATION, &m_cpDictationGrammar);
		//if (SUCCEEDED(hr))
		//{
		//	hr = m_cpDictationGrammar->LoadDictation(NULL, SPLO_STATIC);//加载词典
		//}

		//C&C命令式，此时语法文件使用xml格式
		ullGrammerID = 1000;
		hr = m_cpRecoCtxt->CreateGrammar(ullGrammerID, &m_cpCmdGrammar);

		WCHAR wszXMLFile[20] = L"";//加载语法
		MultiByteToWideChar(CP_ACP, 0, (LPCSTR)"CmdCtrl.xml", -1, wszXMLFile, 256);//ANSI转UNINCODE
		hr = m_cpCmdGrammar->LoadCmdFromFile(wszXMLFile, SPLO_DYNAMIC);


		//MessageBox(NULL, (LPCWSTR)L"语音识别已启动！", (LPCWSTR)L"提示", MB_CANCELTRYCONTINUE );
		//激活语法进行识别
		//hr = m_cpDictationGrammar->SetDictationState(SPRS_ACTIVE);//dictation
		hr = m_cpCmdGrammar->SetRuleState(NULL, NULL, SPRS_ACTIVE);//C&C
		hr = m_cpRecoEngine->SetRecoState(SPRST_ACTIVE);
		
	}

	else
	{
		MessageBox(NULL, (LPCWSTR)L"语音识别引擎启动出错！", (LPCWSTR)L"警告", MB_OK);
		exit(0);
	}
		

	//释放com资源
	::CoUninitialize();
	//hr = m_cpCmdGrammar->SetRuleState(NULL, NULL, SPRS_INACTIVE);//C&C


}


