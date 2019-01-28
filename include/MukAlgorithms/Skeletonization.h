#pragma once
#include "ITKWrapper.h"

namespace gris
{
  namespace muk
  {
    /** \brief Computes a skeletonization by iteratively using morphological binary operations. 

      Extremely time consuming!
      see Skeletonization2.h
    */
    class MUK_ALGO_API Skeletonization : public ItkImageToImageWrapper<ImageInt3D, ImageInt3D>
    {
      public:
        Skeletonization();

      public:
        static  const char* s_name()       { return "Skeletonization"; };
        virtual const char* name()   const { return s_name(); }
    };

  } // namespace muk
} // gris