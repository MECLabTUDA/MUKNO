#pragma once

#ifdef MUK_CONVEX_OPTIMIZATION_CREATE_DLL
#define MUK_CONV_OPT_API __declspec (dllexport)
#else
#define MUK_CONV_OPT_API __declspec (dllimport)
#endif
