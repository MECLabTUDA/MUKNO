#pragma once

//#ifdef MUK_QT_CREATE_DLL
//#define MUK_QT_API __declspec (dllexport)
//#else
//#define MUK_QT_API __declspec (dllimport)
//#endif
#include <qcompilerdetection.h>

#ifdef MUK_QT_CREATE_DLL
#define MUK_QT_API Q_DECL_EXPORT
#else
#define MUK_QT_API Q_DECL_IMPORT
#endif


