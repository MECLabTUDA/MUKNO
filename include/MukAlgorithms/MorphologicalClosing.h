#pragma once
#include "ITKWrapper.h"

namespace gris
{
  namespace muk
  {
    /** \brief Creates a binary/Segmentation image by thresholding at a lower and/or upper limit.
    */
    class MUK_ALGO_API MorphologicalClosing : public ItkImageToImageWrapper<ImageInt3D, ImageInt3D>
    {
      public:
        MorphologicalClosing();

      public:
        static  const char* s_name()       { return "MorphologicalClosing"; };
        virtual const char* name()   const { return s_name(); }

      private:
        void  setSize(const Vec3i&);
        Vec3i getSize() const;
    };

  } // namespace muk
} // gris