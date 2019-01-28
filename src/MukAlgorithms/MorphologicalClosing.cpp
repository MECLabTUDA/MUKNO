#include "private/muk.pch"
#include "MorphologicalClosing.h"
#include "AlgorithmFactory.h"

#include "MukImaging/MukImage.h"
#include "MukImaging/muk_imaging_tools.h"

#include <itkBinaryBallStructuringElement.h>
#include <itkBinaryMorphologicalClosingImageFilter.h>

namespace
{
  using namespace gris::muk;
  using El   = itk::BinaryBallStructuringElement<MukPixel, Dim3D>;
  using Type = itk::BinaryMorphologicalClosingImageFilter<ImageInt3D, ImageInt3D, El>;
}

namespace gris
{
  namespace muk
  {
    REGISTER_ALGORITHM(MorphologicalClosing);

    
    /**
    */
    MorphologicalClosing::MorphologicalClosing()
    {
      mpFilter = make_itk<Type>();
      mDspType = enDisplayOverlay3D;
      mInputPortTypes.push_back(enImageInt3D);
      mOutputPortTypes.push_back(enImageInt3D);

      setSize(Vec3i(1));

      auto* ptr = dynamic_cast<Type*>(mpFilter.GetPointer());
      ptr->SetForegroundValue(1);
      declareProperty<Vec3i>("Radius",
        [=] (const Vec3i& p) { this->setSize(p); },
        [=]                  { return this->getSize(); });
    }

    void  MorphologicalClosing::setSize(const Vec3i& p)
    {
      El el; 
      itk::Size<3> size; 
      for (int i(0); i<3; ++i)
        size[i] = p[i];
      el.SetRadius(size); 
      el.CreateStructuringElement();
      static_cast<Type*>(mpFilter.GetPointer())->SetKernel(el);
    }

    Vec3i MorphologicalClosing::getSize() const
    {
      auto& kernel = static_cast<Type*>(mpFilter.GetPointer())->GetKernel();
      auto size = kernel.GetRadius();
      Vec3i result;
      for (int i(0); i<3; ++i)
        result[i] = size[i];
      return result;
    }
  }
}