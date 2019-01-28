#pragma once

#ifdef MUK_ALGORITHMS_CREATE_DLL
#define MUK_ALGO_API __declspec (dllexport)
#else
#define MUK_ALGO_API __declspec (dllimport)
#endif