#pragma once

#ifdef MUK_COMMON_CREATE_DLL
  #define MUK_COMMON_API __declspec (dllexport)
  #define MUK_TEMPLATE
#else
  #define MUK_COMMON_API __declspec (dllimport)
  #define MUK_TEMPLATE extern
#endif
