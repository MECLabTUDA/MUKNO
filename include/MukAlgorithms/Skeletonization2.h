#pragma once
#include "ITKWrapper.h"

namespace gris
{
  namespace muk
  {
    /** \brief Computes a skeletonization by non maximum suppression.
    */
    class MUK_ALGO_API Skeletonization2 : public ItkImageToImageWrapper<ImageInt3D, ImageInt3D>
    {
      public:
        Skeletonization2();

      public:
        static  const char* s_name()       { return "Skeletonization2"; };
        virtual const char* name()   const { return s_name(); }
    };
  } // namespace muk
} // grsi