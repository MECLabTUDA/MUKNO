#pragma once

#ifdef MUK_NAVIGATION_CREATE_DLL
#define MUK_NAVI_API __declspec (dllexport)
#else
#define MUK_NAVI_API __declspec (dllimport)
#endif