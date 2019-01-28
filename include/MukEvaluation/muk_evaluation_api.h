#pragma once

#ifdef MUK_EVALUATION_CREATE_DLL
#define MUK_EVAL_API __declspec (dllexport)
#else
#define MUK_EVAL_API __declspec (dllimport)
#endif