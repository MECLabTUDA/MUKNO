#include "private/muk.pch"
#include "TensorflowGraph.h"

#include "private/ComputationalGraph.h"
//#include "helper.h"

#include "MukAlgorithms/AlgorithmFactory.h"

#include "MukImaging/MukImage.h"
#include "MukImaging/muk_imaging_tools.h"

#include <itkImageAlgorithm.h>

#include <boost/filesystem.hpp>

namespace gris
{
  namespace muk
  {
    REGISTER_ALGORITHM(TensorflowGraph);

    struct TensorflowGraph::Impl
    {
      ImageInt3D* inputImage = nullptr;
      ImageInt3D::Pointer outputImage;
      
      std::string graphfile = "";
      std::string outputOp = "separable_conv2d_17/truediv";
      int dim = 2; // 0 for x, 1 for y 2 for z;
      int batchSize = 1;
    };

    /* \brief
    */
    TensorflowGraph::TensorflowGraph()
      : mp(std::make_unique<Impl>())
    {
      mDspType = enDisplayOverlay3D;

      mInputPortTypes.push_back(enImageInt3D);
      mOutputPortTypes.push_back(enImageInt3D);

      mp->outputImage = make_itk<ImageInt3D>();

      declareProperty<std::string>("GraphFile",
        [=](auto val) { mp->graphfile = val; },
        [=]() { return mp->graphfile; });
      declareProperty<std::string>("GraphOutputOperation",
        [=](auto val) { mp->outputOp = val; },
        [=]() { return mp->outputOp; });
      declareProperty<int>("BatchSize",
        [=](auto val) { mp->batchSize = val; },
        [=]() { return mp->batchSize; });
      declareProperty<int>("Slice-Direction",
        [=](auto val) { mp->dim = val; },
        [=]() { return mp->dim; });
    }

    /** \brief
    */
    void  TensorflowGraph::setInput(unsigned int portId, void* pDataType)
    {
      mp->inputImage = static_cast<ImageInt3D*>(pDataType);
    }

    /**\brief
    */
    void* TensorflowGraph::getOutput(unsigned int portId)
    {
      return mp->outputImage;
    }


    /**
    */
    void TensorflowGraph::update()
    {
      namespace fs = boost::filesystem;
	    if (! fs::is_regular_file(mp->graphfile))
        throw MUK_EXCEPTION("graph file does not exist", mp->graphfile.c_str());

      //auto result = gris::predict(*(mp->inputImage), std::string(mp->graphfile), mp->OutputOp, mp->batchSize);
      ComputationalGraph graph;
      graph.readGraph(mp->graphfile);
      graph.setBatchSize(mp->batchSize);
      graph.setOutputOperation(mp->outputOp);
      ComputationalGraph::EnDirection dir = mp->dim == 0 ? ComputationalGraph::enX : mp->dim == 1 ? ComputationalGraph::enY : ComputationalGraph::enZ;
      graph.setDirection(dir);
      auto result = graph.predict(*mp->inputImage);

      allocateFromSrc(*mp->inputImage, *mp->outputImage);
      auto region = result->GetLargestPossibleRegion();
      itk::ImageAlgorithm::Copy(result.GetPointer(), mp->outputImage.GetPointer(), region, region);
    }

  } // namespace muk
} // namespace gris