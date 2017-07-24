//------------------------------------------------------------------------------
// <copyright file="BodyBasics.h" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

#pragma once

#include "resource.h"
#include "mmsystem.h" //导入声音头文件
#include <queue>
#include <string>
#include <sapi.h> //导入语音头文件
#include <sphelper.h>//导入语音识别头文件
#include <atlstr.h>


using namespace std;


#pragma comment(lib,"winmm.lib") //导入声音头文件库
#pragma comment(lib,"sapi.lib") //导入语音头文件库
#pragma once
const int WM_RECORD = WM_USER + 100;


class CBodyBasics
{


public:
    /// <summary>
    /// Constructor
    /// </summary>
    CBodyBasics();

    /// <summary>
    /// Destructor
    /// </summary>
    ~CBodyBasics();

	
	
    /// <summary>
    /// Handles window messages, passes most to the class instance to handle
    /// </summary>
    /// <param name="hWnd">window message is for</param>
    /// <param name="uMsg">message</param>
    /// <param name="wParam">message data</param>
    /// <param name="lParam">additional message data</param>
    /// <returns>result of message processing</returns>
    static LRESULT CALLBACK MessageRouter(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam);

    /// <summary>
    /// Handle windows messages for a class instance
    /// </summary>
    /// <param name="hWnd">window message is for</param>
    /// <param name="uMsg">message</param>
    /// <param name="wParam">message data</param>
    /// <param name="lParam">additional message data</param>
    /// <returns>result of message processing</returns>
    LRESULT CALLBACK        DlgProc(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam);

    /// <summary>
    /// Creates the main window and begins processing
    /// </summary>
    /// <param name="hInstance"></param>
    /// <param name="nCmdShow"></param>
    int                     Run(HINSTANCE hInstance, int nCmdShow);

	
	CComPtr<ISpRecognizer>m_cpRecoEngine;// 语音识别引擎(recognition)的接口。
	CComPtr<ISpRecoContext>m_cpRecoCtxt;// 识别引擎上下文(context)的接口。
	CComPtr<ISpRecoGrammar>m_cpCmdGrammar;// 识别文法(grammar)的接口。
	CComPtr<ISpStream>m_cpInputStream;// 流()的接口。
	CComPtr<ISpObjectToken>m_cpToken;// 语音特征的(token)接口。
	CComPtr<ISpAudio>m_cpAudio;// 音频(Audio)的接口。(用来保存原来默认的输入流)
	ULONGLONG  ullGrammerID;
	

	

private:
	HWND                    m_hWnd;


    bool                    m_bGotReco;//语音识别相关
	void                    MSSSpeak(LPCTSTR speakContent);//文字转语音
	void                    MSSListen();//语音转文字

};

