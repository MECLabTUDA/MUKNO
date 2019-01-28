#pragma once
#include "muk_appmodels_api.h"

#include "BaseModel.h"

#include <list>

namespace gris
{
  namespace muk
  {

    class MUK_APP_API LocalEnvironment : public BaseModel
    {
      public:
        LocalEnvironment();

      public:
        static  const char* s_name() { return "LocalEnvironment"; }
        virtual const char* name() const { return s_name(); }

      public:
        void    setMaxSize(size_t size);
        size_t  getMaxSize()             const { return mMaxSize; };
        void                setRawDataPath(const std::string& str)         { mRawDataPath = str; }
        const std::string&  getRawDataPath()                       const   { return mRawDataPath; }
        void                setFileName(const std::string& str)            { mFileName = str; }
        const std::string&  getFileName()                          const   { return mFileName; }
        void                setLastSceneFiles(const std::list<std::string>& strs)            { mLastSceneFiles = strs; }
        const std::list<std::string>& getLastSceneFiles()          const   { return mLastSceneFiles; }
        void                setBinaryDirectory(const std::string& str)     { mBinaryDirectory = str; }
        const std::string&  getBinaryDirectory()                   const   { return mBinaryDirectory; }

      public:
        void setAsLastFilename(const std::string& filename);

      public:
        void save() const;
        void load();

      private:
        size_t      mMaxSize;
        std::string mRawDataPath;
        std::string mBinaryDirectory;
        std::string mFileName;
        std::list<std::string> mLastSceneFiles;
    };

  }
}
