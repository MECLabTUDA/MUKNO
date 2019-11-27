#include "private/muk.pch"

#include "private/ComputationalGraph.h"

#include "MukCommon/MukException.h"

#include "MukImaging/muk_imaging_tools.h"

#include <itkExtractImageFilter.h>
#include <itkImageRegionIterator.h>
#include <itkImageLinearIteratorWithIndex.h>

namespace
{
  struct Batch 
  {
    int start;
    int end;
  };

  void deleteSession2(TF_Session*);
  void deleteSession(TF_Session*, TF_Status*);
  void free_buffer(void* data, size_t length);
  void deallocateTensor(void* data, std::size_t, void*);
  gris::muk::ImageInt2D::Pointer transposeImage(const gris::muk::ImageInt2D& img);
  int labelPixel(const float* data, const itk::Index<2>& idx, int numOfLabels, int slice_idx, unsigned int dimX, unsigned int dimY);
}

namespace gris
{
namespace muk
{
  /** \brief
  */
  ComputationalGraph::ComputationalGraph()
    : mpStatus(TF_Status_Ptr(TF_NewStatus(), TF_DeleteStatus))
    , mpGraph(TF_Graph_Ptr(TF_NewGraph(), TF_DeleteGraph))
    , mpSession(nullptr, [&] (TF_Session* p) -> void { deleteSession(p, mpStatus.get()); })
    , mBatchSize(1)
    , mOutputOperation("truediv")
  {
  }

  /** \brief
  */
  ComputationalGraph::~ComputationalGraph()
  {
    mpSession.reset();
    mpGraph.reset();
    mpStatus.reset();
  }

  /** \brief
  */
  void ComputationalGraph::readGraph(const std::string& fn)
  {
    //TF_ImportGraphDefOptions* opts = TF_NewImportGraphDefOptions();
    auto pOptions = TF_ImportGraphDefOptions_Ptr(TF_NewImportGraphDefOptions(), TF_DeleteImportGraphDefOptions);
    auto pBuffer   = TF_Buffer_Ptr(TF_NewBuffer(), TF_DeleteBuffer);
    // read graph def from file
    //auto pGraph_def = read_file(fn);
    {
      // open file
      FILE* f = fopen(fn.c_str(), "rb");
      // read file length
      fseek(f, 0, SEEK_END);
      long fsize = ftell(f);
      fseek(f, 0, SEEK_SET);  //same as rewind(f);                                            
                              // allocate data and read 
      void* data = malloc(fsize);
      fread(data, fsize, 1, f);
      pBuffer->data = data;
      pBuffer->length = fsize;
      pBuffer->data_deallocator = free_buffer;
      fclose(f);
    }
    // import graph from graph def
    TF_GraphImportGraphDef(mpGraph.get(), pBuffer.get(), pOptions.get(), mpStatus.get());
    // clean up
    //TF_DeleteImportGraphDefOptions(opts);
    // Error handling for import 
    if (TF_GetCode(mpStatus.get()) != TF_OK)
      throw MUK_EXCEPTION("ERROR: Unable to import graph ", "Errorcode 1");
  }


  /** \brief
  */
  ImageInt3D::Pointer ComputationalGraph::predict(const ImageInt3D& img)
  {
    auto* graph  = mpGraph.get();
    auto* status = mpStatus.get();
    // ------------------------------------------------------------
    // inits 
    auto pOptions = TF_SessionOptions_Ptr(TF_NewSessionOptions(), TF_DeleteSessionOptions);
    auto* options = pOptions.get();
    //status = TF_NewStatus();
    //TF_SessionOptions* options = TF_NewSessionOptions();
    // options to set allow_growth = true
    // if other options are desired use Python to generate them in byte code!
    // Python code for this:
    //  gpu_options = tf.GPUOptions(allow_growth = True)
    //  config = tf.ConfigProto()
    //  config.gpu_options.allow_growth = True # additional options canbe set
    //  serialized = config.SerializeToString()
    //  print(list(map(hex, serialized)))#['0x32', '0x2', '0x20', '0x1']
    uint8_t config[11] = { 0x32, 0x2, 0x20, 0x1 };
    TF_SetConfig(options, (void*)config, 11, status);
    auto deleter = [&] (TF_Session* p) -> void { deleteSession(p, status); };
    auto sess = TF_Session_Ptr(TF_NewSession(graph, options, status), deleter);
    auto* session = sess.get();
    //// make Session
    //TF_Session* session = TF_NewSession(graph, options, status);
    //TF_DeleteSessionOptions(options);
    //return session;
    if (TF_GetCode(status) != TF_OK)
      throw MUK_EXCEPTION("ERROR: Unable to create Session ", "Errorcode 2");

    LOG_LINE << "   Successfully imported graph";
    // ------------------------------------------------------------
    // grab input node operation
    // if the network takes multiple inputs by indexing them from 0 to n inputs and stack them in a vector
    TF_Output input_op = TF_Output{ TF_GraphOperationByName(graph, "input_1"), 0 };
    // Error handling if input is not in graph
    if (input_op.oper == nullptr) 
      throw MUK_EXCEPTION("ERROR: Can't init input_op", "Errorcode 3");
    // ------------------------------------------------------------
    // Grab output node operation 
    // if the network gives multiple outputs by indexing them from 0 to n outputs and stack them in a vector
    //TF_Output out_op = { TF_GraphOperationByName(graph, "conv2d_27/truediv"), 0 };
    //TF_Output out_op = { TF_GraphOperationByName(graph, "input_1"), 0 };
    LOG_LINE << "   Searching for Output OP: " << mOutputOperation;
    TF_Output out_op = TF_Output{ TF_GraphOperationByName(graph, mOutputOperation.c_str()), 0 };
    // Error handling if output is not in graph
    if (out_op.oper == nullptr)
      throw MUK_EXCEPTION("ERROR: Can't init out_op", "Errorcode 4");
    // ------------------------------------------------------------    
    // allocate outputImage 
    ImageInt3D::RegionType outputRegion = img.GetLargestPossibleRegion();
    auto itkImageOut = make_itk<ImageInt3D>();
    itkImageOut->SetRegions(outputRegion);
    itkImageOut->Allocate();

    const int num_dims = TF_GraphGetTensorNumDims(graph, out_op, status);
    std::vector<std::int64_t> dims(num_dims);
    TF_GraphGetTensorShape(mpGraph.get(), out_op, dims.data(), num_dims, status);
    if (TF_GetCode(status) != TF_OK)
      throw MUK_EXCEPTION("ERROR: could not get number of Labels", "Errorcode 5");
    
    // Set number of Labels equal to last index of output tensor assuming[?,H,W,C] structur of softmax layer
    const auto numOfLabels = dims[num_dims - 1];
    int idx = mDirection;
    const auto numOfSlices = static_cast<int>(img.GetLargestPossibleRegion().GetSize()[idx]);
    //LOG_LINE << "num slices " << numOfSlices;
    // make Batch size depends on used network!! 
    // for our Unet it is 64 and slices 50-114 contain should contain segmentations
    std::vector<Batch> batches;
    for (int slice=0; slice < numOfSlices; slice += mBatchSize) 
      batches.push_back({ slice, std::min(slice + mBatchSize, numOfSlices) });

    for(const auto& b : batches)
    {
      LOG_LINE << "   Batch Slices (" << b.start << "-" << b.end << ")";
      //LOG_LINE << "   ITK to Tensor";
      auto tensorPtr = batchToTensor(img, b.start, b.end);
      auto* tensor = tensorPtr.get();
      TF_Tensor* output_tensor = nullptr;
      auto output_tensor_ptr = TF_Tensor_Ptr(output_tensor, TF_DeleteTensor);
      // Run a Session
      TF_SessionRun(session,
        nullptr, // Run options.
        &input_op, &tensor, 1, // Input tensors, input tensor values, number of inputs.
        &out_op, &output_tensor, 1, // Output tensors, output tensor values, number of outputs.
        nullptr, 0, // Target operations, number of targets.
        nullptr, // Run metadata.
        status // Output status.
      );

      // Error handling for Run Session
      if (TF_GetCode(status) != TF_OK)
        throw MUK_EXCEPTION("ERROR: run session", "Errorcode 6");
      //LOG_LINE << "   Tensor to ITK";
      // convert TF_Tensor to float* 
      const auto output_data = static_cast<float*>(TF_TensorData(output_tensor));
      // insert batch into output image
      itkImageOut = batchFromTensor(output_data, numOfLabels, *itkImageOut, b.start, b.end);
      // reset output tensor
      output_tensor = nullptr;
    }

    if (TF_GetCode(status) != TF_OK)
      throw MUK_EXCEPTION("ERROR: run session", "Errorcode 7");

    TF_CloseSession(session, status);
    if (TF_GetCode(status) != TF_OK) 
      throw MUK_EXCEPTION("ERROR: close session", "Errorcode 8");
      
    return itkImageOut;
  }

  /** \brief Creates Batch input of 2D images from an ImageInt3D image
  *
  * caller is expect to define batch with start and end index in Z dimension
  */
  ComputationalGraph::TF_Tensor_Ptr ComputationalGraph::batchToTensor(const ImageInt3D& img, int start, int end) 
  {
    auto numSlices  = end - start;
    auto imageSize  = img.GetLargestPossibleRegion().GetSize();
    auto imageIndex = img.GetLargestPossibleRegion().GetIndex();
    // Definitions of inits
    auto dim1 = mDirection == enZ ? imageSize[0] : mDirection == enY ? imageSize[0] : imageSize[1];
    auto dim2 = mDirection == enZ ? imageSize[1] : mDirection == enY ? imageSize[2] : imageSize[2];
    std::vector<std::int64_t> dims = { numSlices, static_cast<std::int64_t>(dim1), static_cast<std::int64_t>(dim2), 1};
    // calculate datasize
    std::size_t data_size = sizeof(float);
    for (auto dim : dims)
      data_size *= dim;

    // Pointer contain Image data
    auto* data = static_cast<float*>(std::malloc(data_size));
    if (data == nullptr)
      throw MUK_EXCEPTION_SIMPLE("allocating memory for TF_Tensor failed!");

    int idxExtract = mDirection;
    int sizeImage;
    int sizeLine;
    switch(mDirection)
    {
      case enX: 
        sizeImage = imageSize[2] * imageSize[1]; 
        sizeLine  = imageSize[1];
        break;
      case enY:
        sizeImage = imageSize[2] * imageSize[0]; 
        sizeLine  = imageSize[0];
        break;
      case enZ: 
        sizeImage = imageSize[0] * imageSize[1]; 
        sizeLine  = imageSize[0];
        break;
    };

    for (int numS = 0; numS < numSlices; ++numS) 
    {
      imageSize[mDirection]  = 0; // we extract along z direction
      imageIndex[mDirection] = start + numS;
      // Extracted Region
      ImageInt3D::RegionType desiredRegion;
      desiredRegion.SetSize(imageSize);
      desiredRegion.SetIndex(imageIndex);
      // Extract Image Slice
      auto extractor = make_itk<itk::ExtractImageFilter< ImageInt3D, ImageInt2D >>();
      extractor->SetInput(&img);
      extractor->SetDirectionCollapseToSubmatrix();
      extractor->SetExtractionRegion(desiredRegion);
      extractor->Update();
      mExtractedSize = extractor->GetOutput()->GetLargestPossibleRegion().GetSize();
      // convert itkimage indexing to TF indexing
      auto imgT = transposeImage(*(extractor->GetOutput()));

      auto it = itk::ImageRegionIterator<ImageInt2D>(imgT, imgT->GetRequestedRegion());
      int i = 0;
      // write Image data into Pointers address space
      while (!it.IsAtEnd()) 
      {
        auto idx = it.GetIndex();
        data[numS * sizeImage + idx[0] * sizeLine + idx[1]] = it.Get();
        i++;
        ++it;
      }
    }

    return TF_Tensor_Ptr(
      TF_NewTensor(TF_FLOAT, dims.data(), static_cast<int>(dims.size()), data, data_size, deallocateTensor, nullptr),
      TF_DeleteTensor
    );
  }


  /** \brief loads batch data into ImageInt3D
  *
  * expects a softmax layer as output node
  */
  gris::muk::ImageInt3D::Pointer ComputationalGraph::batchFromTensor(const float* data, int numOfLabels, const gris::muk::ImageInt3D& itkImage, int start, int end)
  { 
    using namespace gris::muk;
    // inits
    auto outputSize  = itkImage.GetLargestPossibleRegion().GetSize();
    auto sliceRegion = itkImage.GetLargestPossibleRegion();
    auto sliceSize   = sliceRegion.GetSize();
    auto sliceIndex  = sliceRegion.GetIndex();
    // set Z dimension to 1 as we extract slice by slice from data
    int idxExtract = mDirection;
    int dimX = mDirection == enZ ? outputSize[0] : mDirection == enY ? outputSize[0] : outputSize[1];
    int dimY = mDirection == enZ ? outputSize[1] : mDirection == enY ? outputSize[2] : outputSize[2];

    sliceSize[idxExtract] = 1;
    sliceRegion.SetSize(sliceSize);
    auto outImage = make_itk<ImageInt3D>();
    outImage->Graft(&itkImage);
    // iterate over each slice in batch
    for (int numS = 0; numS < end - start; numS++)
    {
      // fill 2D image
      auto slice = make_itk<ImageInt2D>();
      {
        auto reg = slice->GetLargestPossibleRegion();
        reg.SetSize(mExtractedSize);
        slice->SetRegions(reg);
        slice->Allocate();
        auto itout = itk::ImageRegionIterator<ImageInt2D>(slice, reg);
        itout.GoToBegin();
        // extract label for each pixel
        while (!itout.IsAtEnd())
        {
          auto idx = itout.GetIndex();
          // argmax of output componets
          itout.Set(labelPixel(data, idx, numOfLabels, numS, dimX, dimY));
          ++itout;
        }
      }
      auto sliceT = transposeImage(*slice);

      sliceIndex[idxExtract] = start + numS;
      sliceRegion.SetIndex(sliceIndex);
      auto itOut = itk::ImageRegionIterator<ImageInt3D>(outImage, sliceRegion);
      auto itIn  = itk::ImageRegionIterator<ImageInt2D>(sliceT, sliceT->GetLargestPossibleRegion());
      itOut.GoToBegin();
      itIn.GoToBegin();
      // extract label for each pixel
      while (!itOut.IsAtEnd())
      {
        itOut.Set(itIn.Get());
        ++itIn;
        ++itOut;
      }
    }
    return outImage;
  }
}
}

namespace
{
  /** \brief
  */
  void free_buffer(void* data, size_t length)
  {
    free(data);
  }

  /** \brief
  */
  void deleteSession(TF_Session* p, TF_Status* status)
  {
    TF_DeleteSession(p, status);
  }

  /** \brief
  */
  void deallocateTensor(void* data, std::size_t, void*)
  {
    std::free(data);
  }

  /**
  */
  itk::SmartPointer<gris::muk::ImageInt2D> transposeImage(const gris::muk::ImageInt2D& img) 
  {
    using namespace gris::muk;
    auto pImg = make_itk<ImageInt2D>();
    // allocate
    {
      auto& target = *pImg;
      auto reg = img.GetBufferedRegion();
      auto size = reg.GetSize();
      std::swap(size[0], size[1]);
      reg.SetSize(size);
      target.SetRegions(reg);
      target.SetOrigin(img.GetOrigin());
      target.SetDirection(img.GetDirection());
      target.SetSpacing(img.GetSpacing());
      target.Allocate();
    }
    // transpose
    {
      auto inputIter  = itk::ImageLinearConstIteratorWithIndex<ImageInt2D>(&img, img.GetRequestedRegion());
      auto outputIter = itk::ImageLinearIteratorWithIndex<ImageInt2D>(pImg, pImg->GetRequestedRegion());
      inputIter.SetDirection(0);
      outputIter.SetDirection(1);
      while (!inputIter.IsAtEnd())
      {
        while (!inputIter.IsAtEndOfLine())
        {
          auto value = inputIter.Get();
          outputIter.Set(value);
          ++inputIter;
          ++outputIter;
        }
        inputIter.NextLine();
        outputIter.NextLine();
      }
    }
    return pImg;
  }

  /** \brief outputs the argmax(label) of pixel
  */
  int labelPixel(const float* data,const itk::Index<2>& idx, int numOfLabels, int slice_idx, unsigned int dimX, unsigned int dimY)
  {
    std::vector<float> label_values;
    float value = 0;
    int arg = -1;
    // argmax of softmax output
    for (int label = 0; label < numOfLabels; label++)
    {
      // pick label with highest value
      if (value <= data[slice_idx * dimX * dimY * numOfLabels + idx[0] * dimY * numOfLabels + idx[1] * numOfLabels + label]) 
      {
        value = data[slice_idx * dimX * dimY * numOfLabels + idx[0] * dimY * numOfLabels + idx[1] * numOfLabels + label];
        arg = label;
      }
    }
    return arg;
  }
}