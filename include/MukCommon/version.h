#pragma once

#include "muk_common_api.h"

#include <windows.h>

namespace gris
{
namespace muk
{
  /**
  */
  struct MUK_COMMON_API LibVersion
  {
    LibVersion() {}
    LibVersion(DWORD& a, DWORD& b, DWORD& c, DWORD& d) : mMajor(a), mMinor(b), mRevision(c), mBuild(d) {}
    LibVersion(int a, int b, int c, int d) : mMajor((DWORD)a), mMinor((DWORD)b), mRevision((DWORD)c), mBuild((DWORD)d) {}
    LibVersion(const char* str);
    
    bool operator==(const LibVersion& o) const
    {
      return mMajor == o.mMajor && mMinor == o.mMinor && mRevision == o.mRevision && mBuild == o.mBuild;
    }
    bool operator!=(const LibVersion& o) const
    {
      return ! (*this==o);
    }

    bool operator<(const LibVersion& o) const
    {
      return mMajor < o.mMajor
        || ( mMajor == o.mMajor && ( mMinor < o.mMinor ||
           ( mMinor == o.mMinor && ( mRevision < o.mRevision)  ||
           ( mRevision == o.mRevision && mBuild < o.mBuild))));
    }

    DWORD mMajor;
    DWORD mMinor;
    DWORD mRevision;
    DWORD mBuild;
  };

  MUK_COMMON_API void getMuknoLibraryVersion(const char* libraryName, DWORD& major, DWORD& minor, DWORD& revision, DWORD& build);
}
}
