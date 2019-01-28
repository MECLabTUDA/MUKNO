#pragma once

#ifdef MUK_IMAGING_CREATE_DLL
#define MUK_IMG_API __declspec (dllexport)
#else
#define MUK_IMG_API __declspec (dllimport)
#endif