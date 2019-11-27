#pragma once

#include "MukImaging/MukImage.h"

#pragma warning (push)
#pragma warning (disable:4190) // TF_NewWhile' has C-linkage specified, but returns UDT 'TF_WhileParams' which is incompatible with C 
extern "C"
{
#include "tensorflow/c_api.h"
}
#pragma warning (pop)

namespace gris
{
  namespace muk
  {
    /** \brief Allows loading a frozen Tensorflow graph and a prediction on itk::Images

      During prediction, it internally starts a Tensorflow session
      The class wraps the C-API of tensorflow and takes care of memory management of TF_<Object>s.
    */
    class ComputationalGraph
    {
      public:
        enum EnDirection
        {
          enX,
          enY,
          enZ
        };

      public:
        ComputationalGraph();
        ~ComputationalGraph();

      public:
        void setOutputOperation(const std::string& str) { mOutputOperation = str; }
        void setBatchSize(int val)                      { mBatchSize = val; }
        void setDirection(EnDirection val)              { mDirection = val; }

      public:
        ImageInt3D::Pointer predict(const ImageInt3D& img);
        void readGraph(const std::string& fn);

      private:
        using TF_Buffer_Ptr   = std::unique_ptr<TF_Buffer, void(*)(TF_Buffer*)>;
        using TF_Status_Ptr   = std::unique_ptr<TF_Status, void(*)(TF_Status*)>;
        using TF_Graph_Ptr    = std::unique_ptr<TF_Graph, void(*)(TF_Graph*)>;
        using TF_Tensor_Ptr   = std::unique_ptr<TF_Tensor, void(*)(TF_Tensor*)>;
        //using TF_Session_Ptr  = std::unique_ptr<TF_Session, void(*)(TF_Session*)>;
        using TF_Session_Ptr  = std::unique_ptr<TF_Session, std::function<void(TF_Session*)>>;

        using TF_ImportGraphDefOptions_Ptr  = std::unique_ptr<TF_ImportGraphDefOptions, void(*)(TF_ImportGraphDefOptions*)>;
        using TF_SessionOptions_Ptr         = std::unique_ptr<TF_SessionOptions, void(*)(TF_SessionOptions*)>;

      private:
        ComputationalGraph::TF_Tensor_Ptr batchToTensor(const ImageInt3D& itkImage, int start, int end);
        gris::muk::ImageInt3D::Pointer    batchFromTensor(const float* data, int numOfLabels, const gris::muk::ImageInt3D& itkImage, int start, int end);
      
      private:
        std::string mOutputOperation;
        int         mBatchSize;
        EnDirection mDirection;

        ImageInt2D::SizeType mExtractedSize; // defined during prediction for use in reconstruction from TF data
        
        TF_Status_Ptr   mpStatus;
        TF_Graph_Ptr    mpGraph;
        TF_Session_Ptr  mpSession;
    };
  }
}