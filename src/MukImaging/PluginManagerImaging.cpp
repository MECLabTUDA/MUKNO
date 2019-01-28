#include "private/muk.pch"
#include "PluginManagerImaging.h"

#include <itkMetaImageIOFactory.h>
#include <itkGDCMImageIOFactory.h>

namespace gris
{
namespace muk
{
  /**
  */
  void PluginManagerImaging::initialize()
  {
    //RegisteredObjectsContainerType registeredIOs = itk::ObjectFactoryBase::CreateAllInstance("itkImageIOBase");

    itk::MetaImageIOFactory::RegisterOneFactory();
    itk::GDCMImageIOFactory::RegisterOneFactory();
  }
}
}