#pragma once
#include "ITKWrapper.h"

#include "MukCommon/muk_common.h"

namespace gris
{
  namespace muk
  {
    /** \brief Class to wrapp itk::SliceImageFileWriter
    */
    class MUK_ALGO_API SliceImageFileWriter : public ItkProcessObjectWrapper<ImageInt3D>
    {
      public:
		    SliceImageFileWriter();

      public:
        virtual void setInput(unsigned int portId, void* pDataType);

      public:
        static const char* s_name() { return "SliceImageFileWriter"; };
        virtual const char* name()  const { return s_name(); }
    };
  }
}