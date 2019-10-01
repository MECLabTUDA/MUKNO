#pragma once
#include "muk_common_api.h"

#include "gstd/dynamicProperty.h"

#include <vtkSmartPointer.h>
class vtkPolyData;

#include <string>

namespace gris
{
  namespace muk
  {
    /** \brief A first representation of an obstacle. 
    
      Features data (as vtkPolyData, so it can be used for visualization as well), a unique identifer name and information if it is active for collision checking.
      Should probably be refactored.
    */
    class MUK_COMMON_API MukObstacle : public gstd::DynamicProperty
    {
      public:
        MukObstacle();
        ~MukObstacle();

      public:
        void                 setName(const std::string& s)                  { mName = s; }
        const std::string&   getName()                              const   { return mName; }
        void                 setFileName(const std::string& fn)             { mFileName = fn; }
        const std::string&   getFileName()                          const   { return mFileName; }
        void                 setActive(bool on)                             { mActive = on; }
        bool                 getActive()                            const   { return mActive; }
        void                 setData(vtkSmartPointer<vtkPolyData> pData)            { mData = pData; }
        vtkSmartPointer<vtkPolyData> getData()                              const   { return mData; }
        
      public:
        void swap(MukObstacle& o);
        void load();
        void save() const;

      private:
        std::string mName;
        std::string mFileName;
        bool        mActive;
        vtkSmartPointer<vtkPolyData> mData;
    };

  }
}
