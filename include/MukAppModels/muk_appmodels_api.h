#pragma once

#ifdef MUK_APPMODELS_CREATE_DLL
#define MUK_APP_API __declspec (dllexport)
#else
#define MUK_APP_API __declspec (dllimport)
#endif
