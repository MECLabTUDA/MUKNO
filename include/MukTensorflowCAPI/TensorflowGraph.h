#pragma once
#include "muk_tensorflow_c_api.h"
#include "AlgorithmWrapper.h"

namespace gris
{
  namespace muk
  {
    /** \brief Class to run a 2D Segmentation network on 3D Itk Input Image
    *
    *   The class is given a Network, which must be frozen into a pb file (use freeze.py).
    *   The Network should have only one Input and one Output Operation.
    *   The name of the Output Operation needs to be given.
    *
    *   @param inputImage  Image of type ImageInt3D to perform Segmentation on. Use SetInput(Image) function to set.
    *   @param GraphFile Filepath to frozen graph file(.pb). Use SetProperty on the Algorithm.
    *   @param GraphOutputOperation String of the desired output node, which should be a softmax layer (eg. "separable_conv2d_17/truediv"). Use SetProperty on the Algorithm.
    *   @param BatchSize Optional property to set the processing batchsize. Default is 1. Use SetProperty on the Algorithm.
    *   @return outputImage Segmentation of the Image. Update() needs to be called before requesting Output with GetOutput().
    */
    class MUK_TF_C_API TensorflowGraph : public AlgorithmWrapper
    {
      public:
        TensorflowGraph();

      public:
        virtual void  setInput(unsigned int portId, void* pDataType);
        virtual void* getOutput(unsigned int portId);
        virtual void  update();

      public:
        static  const char* s_name() { return "TensorflowGraph"; };
        virtual const char* name()   const { return s_name(); }

      private:
        struct Impl;
        std::unique_ptr<Impl> mp;
    };

  } // namespace muk
} // gris