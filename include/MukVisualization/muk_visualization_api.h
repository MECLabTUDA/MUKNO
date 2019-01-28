#pragma once

#ifdef MUK_VISUALIZATION_CREATE_DLL
#define MUK_VIS_API __declspec (dllexport)
#else
#define MUK_VIS_API __declspec (dllimport)
#endif