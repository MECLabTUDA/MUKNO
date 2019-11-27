#pragma once
#include "muk_tensorflow_c_api.h"

namespace gris
{
  namespace muk
  {
    /* \brief Plugin to run a 2D Segmentation network on 3D Itk Input Image
    * To use this plugin a Network must be frozen into a pb file (use freeze.py).
    * The Network should have only one Input and one Output node which have the same size.
    * The name of the Output node needs to be given.
    */
    struct MUK_TF_C_API PluginManagerTensorflowCAPI
    {
      static void initialize();
    };
  }
}