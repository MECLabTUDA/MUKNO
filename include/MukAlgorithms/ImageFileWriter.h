#pragma once
#include "ITKWrapper.h"

#include "MukCommon/muk_common.h"

namespace gris
{
  namespace muk
  {
    /** \brief Class to wrapp itk::ImageFileWriter
    */
    class MUK_ALGO_API ImageFileWriter : public ItkProcessObjectWrapper<ImageInt3D>
    {
      public:
		    ImageFileWriter();

      public:
        virtual void setInput(unsigned int portId, void* pDataType);

        virtual void update();

      public:
        static const char* s_name() { return "ImageFileWriter"; };
        virtual const char* name()  const { return s_name(); }
    };
  }
}