#include "private/muk.pch"
#include "AndOrNot.h"
#include "AlgorithmFactory.h"

#include "MukCommon/MukException.h"

#include "MukImaging/MukImage.h"
#include "MukImaging/muk_imaging_tools.h"

#include <itkAndImageFilter.h>
#include <itkOrImageFilter.h>
#include <itkNotImageFilter.h>


namespace
{
  using namespace gris::muk;
  using And = itk::AndImageFilter<ImageInt3D>;
  using Or  = itk::OrImageFilter <ImageInt3D>;
  using Not = itk::NotImageFilter<ImageInt3D,ImageInt3D>;
}

namespace gris
{
  namespace muk
  {
    REGISTER_ALGORITHM(AndOrNot);

    struct AndOrNot::Impl
    {
      AndOrNot* ptr;
      std::string type;

      ImageInt3D* input1 = nullptr;
      ImageInt3D* input2 = nullptr;
      ImageInt3D::Pointer output;

      void setType(const std::string& str)
      {
        if (str=="and")
        {
          ptr->mpFilter = make_itk<And>();
          auto p = dynamic_cast<And*>(ptr->mpFilter.GetPointer());
          output->Graft(p->GetOutput());
        }
        else if (str=="or")
        {
          ptr->mpFilter = make_itk<Or>();
          auto p = dynamic_cast<Or*>(ptr->mpFilter.GetPointer());
          output->Graft(p->GetOutput());
        }
        else if (str=="not")
        {
          ptr->mpFilter = make_itk<Not>();
          auto p = dynamic_cast<Not*>(ptr->mpFilter.GetPointer());
          output->Graft(p->GetOutput());
        }
        else
        {
          throw MUK_EXCEPTION("Unknown type", "known types: and, or, not");
        }
        type = str;
        setInput();
      }

      void setInput()
      {
        if (type=="and")
        {
          auto p = dynamic_cast<And*>(ptr->mpFilter.GetPointer());
          p->SetInput1(input1);
          p->SetInput2(input2);
          output->Graft(p->GetOutput());
        }
        else if (type=="or")
        {
          auto p = dynamic_cast<Or*>(ptr->mpFilter.GetPointer());
          p->SetInput1(input1);
          p->SetInput2(input2);
          output->Graft(p->GetOutput());
        }
        else if (type=="not")
        {
          auto p = dynamic_cast<Not*>(ptr->mpFilter.GetPointer());
          p->SetInput(input1);
          output->Graft(p->GetOutput());
        }
      }
    };

    /**
    */
    AndOrNot::AndOrNot()
      : mp(std::make_unique<Impl>())
    {
      mp->ptr = this;
      mp->output = make_itk<ImageInt3D>();
      
      mDspType = enDisplayOverlay3D;
      mInputPortTypes.push_back(enImageInt3D);
      mInputPortTypes.push_back(enImageInt3D);
      mOutputPortTypes.push_back(enImageInt3D);

      mp->setType("and");

      declareProperty<std::string>("Type",
        [&] (const std::string& str) { mp->setType(str); },
        [&] ()                       { return mp->type; });
    }

    void AndOrNot::setInput(unsigned int portId, void* pDataType)
    {
      switch (portId)
      {
        case 0:
          mp->input1 = toDataType<ImageInt3D>(pDataType);
          break;
        case 1:
          mp->input2 = toDataType<ImageInt3D>(pDataType);
          break;
      }
      mp->setInput();
    }
  }
}