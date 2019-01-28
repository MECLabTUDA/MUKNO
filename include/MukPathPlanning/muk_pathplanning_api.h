#pragma once

#ifdef MUK_PATH_PLANNING_CREATE_DLL
#define MUK_PP_API __declspec (dllexport)
#else
#define MUK_PP_API __declspec (dllimport)
#endif