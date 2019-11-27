#pragma once

#ifdef MUK_TENSORFLOW_CAPI_CREATE_DLL
#define MUK_TF_C_API __declspec (dllexport)
#else
#define MUK_TF_C_API __declspec (dllimport)
#endif