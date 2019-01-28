#include "private/muk.pch"
#include "version.h"
#include "MukException.h"

#include <boost/format.hpp>

#include <string>
#include <sstream>

namespace gris
{
namespace muk
{
  /**
  */
  LibVersion::LibVersion(const char* str_)
  {
    std::string str(str_);    
    if (std::any_of(str.begin(), str.end(), [&] (const char c) { return ! ( c=='.' || (c>='0' && c <= '9')); } ))
      throw MUK_EXCEPTION("invalid version string", str_);
    int n = static_cast<int>( std::count_if(str.begin(), str.end(), [&] (const char c) { return c == '.'; }) );
    if (n!=3)
      throw MUK_EXCEPTION("invalid version string", str_);
    std::replace(str.begin(), str.end(), '.', ' ');
    std::stringstream iss(str);
    iss >> mMajor;
    iss >> mMinor;
    iss >> mRevision;
    iss >> mBuild;
  }

  /**
  */
  void getMuknoLibraryVersion(const char* libraryName, DWORD& major, DWORD& minor, DWORD& revision, DWORD& build)
  {
    DWORD               dwSize              = 0;
    BYTE                *pbVersionInfo      = NULL;
    VS_FIXEDFILEINFO    *pFileInfo          = NULL;
    UINT                puLenFileInfo       = 0;

    // get the version info for the file requested
    dwSize = GetFileVersionInfoSize( libraryName, NULL );
    BOOL valid = dwSize != 0;
    if (valid)
    {
      pbVersionInfo = new BYTE[ dwSize ];
      if (valid = GetFileVersionInfo( libraryName, 0, dwSize, pbVersionInfo))
      {
        if (valid = GetFileVersionInfo(libraryName, 0, dwSize, pbVersionInfo))
        {
          if (valid = VerQueryValue(pbVersionInfo, TEXT("\\"), (LPVOID*)&pFileInfo, &puLenFileInfo))
          {
            major     = (pFileInfo->dwFileVersionMS >> 16) & 0xffff;
            minor     = (pFileInfo->dwFileVersionMS >>  0) & 0xffff;
            revision  = (pFileInfo->dwFileVersionLS >> 16) & 0xffff;
            build     = (pFileInfo->dwFileVersionLS >>  0) & 0xffff;
          }
        }        
      }
      delete[] pbVersionInfo;      
    }
    if (!valid)
    {
      throw MUK_EXCEPTION((boost::format("Checking Library %s failed!") % libraryName).str().c_str(), (boost::format("Error Code: %d") % GetLastError()).str().c_str());
    }
  }
}
}