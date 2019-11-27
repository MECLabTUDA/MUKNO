#include "private/muk.pch"
#include "helper.h"

#include "MukCommon/MukException.h"
#include "MukImaging/MukImage.h"
#include "MukImaging/muk_imaging_tools.h"

#include <itkExtractImageFilter.h>
#include <itkFlipImageFilter.h>
#include <itkImageFileReader.h>
#include <itkImageFileWriter.h>
#include <itkImageRegionIterator.h>
#include <itkImageLinearIteratorWithIndex.h>

#include <boost/filesystem.hpp>


namespace
{
  using namespace gris::muk;
  namespace fs = boost::filesystem;
}

namespace gris
{
  /** \brief
  */
  struct batch 
  {
    int start;
    int end;
  };
  /** \brief Function to read pb Files
  * caller receives ownership of TF_Buffer
  */
  TF_Buffer* read_file(const char* file)
  {
    // open file
    FILE *f = fopen(file, "rb");
    // read file length
    fseek(f, 0, SEEK_END);
    long fsize = ftell(f);
    fseek(f, 0, SEEK_SET);  //same as rewind(f);                                            
    // allocate data and read 
    void* data = malloc(fsize);
    fread(data, fsize, 1, f);
    fclose(f);
    // construct TF_Buffer
    TF_Buffer* buf = TF_NewBuffer();
    buf->data = data;
    buf->length = fsize;
    buf->data_deallocator = free_buffer;
    return buf;
  }

  /** \brief cpp deallocator for TF_Buffer
  */
  void free_buffer(void* data, size_t length)
  {
    free(data);
  }

  /** \brief cpp deallocator for TF_Tensor
  */
  static void DeallocateTensor(void* data, std::size_t, void*)
  {
    std::free(data);
    LOG_LINE << "Deallocate tensor";
  }

  /** \brief loads ImageInt3D from file
  */
  itk::SmartPointer<ImageInt3D> loadImage(const char* file)
  {
    auto reader = make_itk<itk::ImageFileReader<ImageInt3D>>();
    reader->SetFileName(file);
    reader->Update();
    return reader->GetOutput();
  }

  /** \brief Transposes Image
  * used to convert itk indexing(HWC) between Tensorflow indexing(CHW)
  */
  itk::SmartPointer<gris::muk::ImageInt3D> transposeImage(const ImageInt3D &itkImage) 
  {
    auto pImg = make_itk<ImageInt3D>();
    {
      ImageInt3D::ConstPointer inputImage(&itkImage);
      allocateFromSrc(*inputImage, *pImg);

      itk::ImageLinearConstIteratorWithIndex<ImageInt3D>  inputIter(inputImage, inputImage->GetRequestedRegion());
      itk::ImageLinearIteratorWithIndex<ImageInt3D>  outputIter(pImg, pImg->GetRequestedRegion());
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
      ///* Leaving this for validation purpose */
      //auto writer = make_itk<itk::ImageFileWriter<ImageInt3D>>();
      //writer->SetInput(pImg);
      //writer->SetFileName("../../../../repo/MUKNO_II/data/P01/in_01.mhd");
      //writer->Update();
    }
    return pImg;
  }

  /** \brief Creates Batch input of 2D images from an ImageInt3D image
  * caller is expect to define batch with start and end index in Z dimension
  * caller receives ownership of TF_Tensor*
  */
  TF_Tensor* makeBatch(const ImageInt3D &itkImage, int start, int end) 
  {
    auto numSlices  = end - start;
    auto imageSize  = itkImage.GetLargestPossibleRegion().GetSize();
    auto imageIndex = itkImage.GetLargestPossibleRegion().GetIndex();
    // Definitions of inits
    std::vector<std::int64_t> dims = { numSlices, static_cast<std::int64_t>(imageSize[0]), static_cast<std::int64_t>(imageSize[1]),1 };
    // calculate datasize
    std::size_t data_size = sizeof(float);
    for (auto dim : dims)
      data_size *= dim;

    // Pointer contain Image data
    auto data = static_cast<float*>(std::malloc(data_size));
    for (int numS = 0; numS < numSlices; numS++) 
    {
      imageSize[2] = 1; // we extract along z direction
      imageIndex[2] = start + numS;
      // Extracted Region
      ImageInt3D::RegionType desiredRegion;
      desiredRegion.SetSize(imageSize);
      desiredRegion.SetIndex(imageIndex);
      // Extract Image Slice
      auto extractor = make_itk<itk::ExtractImageFilter< ImageInt3D, ImageInt3D >>();
      extractor->SetInput(&itkImage);
      extractor->SetDirectionCollapseToSubmatrix();
      extractor->SetExtractionRegion(desiredRegion);
      extractor->Update();
      // convert itkimage indexing to TF indexing
      auto ImageT = transposeImage(*(extractor->GetOutput()));

      itk::ImageRegionIterator<ImageInt3D>  it(ImageT, ImageT->GetRequestedRegion());
      int i = 0;
      // write Image data into Pointers address space
      while (!it.IsAtEnd()) 
      {
        auto idx = it.GetIndex();
        data[numS * imageSize[0] * imageSize[1] + idx[0] * imageSize[0] + idx[1]] = it.Get();
        i++;
        ++it;
      }
    }
    // convert Image Data to TF_Tensor
    TF_Tensor* tensor = TF_NewTensor(TF_FLOAT,
      dims.data(), static_cast<int>(dims.size()),
      data, data_size,
      DeallocateTensor, nullptr);
    return tensor;
  }

  /** \brief Imports Graph from file
  * caller receives ownership of TF_Graph* and is expect to do error handling of status
  */
  TF_Graph* importFrozenGraph(const char* file, TF_Status* status) 
  {
    // inits
    status = TF_NewStatus();
    TF_Graph* graph = TF_NewGraph();
    TF_ImportGraphDefOptions* opts = TF_NewImportGraphDefOptions();
    // read graph def from file
    TF_Buffer* graph_def = read_file(file);
    // import graph from graph def
    TF_GraphImportGraphDef(graph, graph_def, opts, status);
    // clean up
    TF_DeleteImportGraphDefOptions(opts);
    TF_DeleteBuffer(graph_def);
    return graph;
  }

  /** \brief Creates TF_Session* with Option:allow_growth = True
  * caller receives ownership of TF_Session*
  */
  TF_Session* createSession(TF_Graph* graph, TF_Status* status)
  {
    // inits
    status = TF_NewStatus();
    TF_SessionOptions* options = TF_NewSessionOptions();
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
    // make Session
    TF_Session* session = TF_NewSession(graph, options, status);
    TF_DeleteSessionOptions(options);
    return session;
  }

  /** \brief outputs the argmax(label) of pixel
  */
  int labelPixel(const float* data,const itk::Index<3> &idx, int &numOfLabels, int &slice_idx, const itk::Size<3> &outputSize)
  {
    // inits
    std::vector<float> label_values;
    float value = 0;
    int arg = -1;
    // argmax of softmax output
    for (int label = 0; label < numOfLabels; label++)
    {
      // pick label with highest value
      if (value <= data[slice_idx * outputSize[0] * outputSize[1] * numOfLabels + idx[0] * outputSize[1] * numOfLabels + idx[1] * numOfLabels + label]) 
      {
        value = data[slice_idx * outputSize[0] * outputSize[1] * numOfLabels + idx[0] * outputSize[1] * numOfLabels + idx[1] * numOfLabels + label];
        arg = label;
      }
    }
    return arg;
  }
    
  /** \brief writes Image to filepath as out[_number].mhd
  * default is out.mhd
  */
  void writeImage(const gris::muk::ImageInt3D &itkImage,const std::string &filepath, int number = -1)
  {
    auto writer = make_itk<itk::ImageFileWriter<ImageInt3D>>();
    writer->SetInput(&itkImage);
    if (number == -1)
    {
      writer->SetFileName(filepath + "out.mhd");
    }
    else 
    {
      writer->SetFileName(filepath + "out_" + std::to_string(number) + ".mhd");
    }
    writer->Update();
  }

  /** \brief loads batch data into ImageInt3D
  * expects a softmax layer as output node
  */
  itk::SmartPointer<gris::muk::ImageInt3D> batchtoLabelImage(const float* data, int numOfLabels, const ImageInt3D &itkImage, int &start, int &end)
  {
    // inits
    auto outputSize = itkImage.GetLargestPossibleRegion().GetSize();
    auto sliceRegion = itkImage.GetLargestPossibleRegion();
    auto sliceSize = sliceRegion.GetSize();
    auto sliceIndex = sliceRegion.GetIndex();
    // set Z dimension to 1 as we extract slice by slice from data
    sliceSize[2] = 1;
    sliceRegion.SetSize(sliceSize);
    ImageInt3D::Pointer outImage = ImageInt3D::New();
    outImage->Graft(&itkImage);
    // iterate over each slice in batch
    for (int numS = 0; numS < end - start; numS++)
    {
      sliceIndex[2] = start + numS;
      sliceRegion.SetIndex(sliceIndex);
      auto itout = itk::ImageRegionIterator<ImageInt3D>(outImage, sliceRegion);
      itout.GoToBegin();
      // extract label for each pixel
      while (!itout.IsAtEnd())
      {
        auto idx = itout.GetIndex();
        // argmax of output componets
        itout.Set(gris::labelPixel(data, idx, numOfLabels, numS, outputSize));
        ++itout;
      }
    }
    return outImage;
  }

  /** \brief writes ImageInt3D images with all their channels to filepath
  */
  void batchtoImages(float* &data, const itk::Size<3> &outputSize, int &start, int &end, const std::string &filepath) 
  {
    // 
    ImageInt3D::RegionType outputRegion;
    outputRegion.SetSize(outputSize);

    ImageInt3D::Pointer itkImageOut = ImageInt3D::New();
    itkImageOut->SetRegions(outputRegion);
    itkImageOut->Allocate();

    for (int numS = 0; numS < end-start; numS++) {
     itk::ImageRegionIterator<ImageInt3D>  itout(itkImageOut, itkImageOut->GetRequestedRegion());
     itout.GoToBegin();
     while (!itout.IsAtEnd())
     {
      auto idx = itout.GetIndex();
      itout.Set(data[numS * outputSize[0] * outputSize[1] * outputSize[2] +  idx[0] * outputSize[1] * outputSize[2] + idx[1] * outputSize[2] + idx[2]] * 5);
      ++itout;
     }
     gris::writeImage(*itkImageOut, filepath, numS + start);
    }
  }

  /** \brief predicts Segmentation with tensorflow graph
  */
  itk::SmartPointer<gris::muk::ImageInt3D> predict(const gris::muk::ImageInt3D &itkImage, const std::string &graphFilename, const std::string &outputOP, const int &batchSize) 
  {
    try
    {
      {
        auto str = gris::GetGrisLogger().getFilename();
        fs::absolute(str);
        if (fs::exists(str))
          fs::remove(str);
      }
      gris::GetGrisLogger().setFileLogging(true);

      int numOfSlices = itkImage.GetLargestPossibleRegion().GetSize()[2];
      // make Batch size depends on used network!! 
      // for our Unet it is 64 and slices 50-114 contain should contain segmentations
      std::vector<batch> batches;
      for (int slice = 0; slice < numOfSlices; slice += batchSize) {
        batches.push_back({ slice, std::min(slice + batchSize, numOfSlices) });
      }
      
      // Check frozen graph bexistence
      if (!fs::is_regular(graphFilename))
        throw MUK_EXCEPTION("file does not exist", fs::absolute(graphFilename).string().c_str());
      TF_Status* status = TF_NewStatus();
      auto graph = gris::importFrozenGraph(graphFilename.c_str(), status);
      // Error handling for import 
      if (TF_GetCode(status) != TF_OK) {
        // clean up
        TF_DeleteStatus(status);
        TF_DeleteGraph(graph);
        throw MUK_EXCEPTION("ERROR: Unable to import graph ", "1");
      }
      // Print graph operations
      gris::PrintOp(graph);

      status = TF_NewStatus();
      TF_Session* sess = gris::createSession(graph, status);

      if (TF_GetCode(status) != TF_OK) {
        TF_DeleteSession(sess, status);
        TF_DeleteStatus(status);
        TF_DeleteGraph(graph);
        throw MUK_EXCEPTION("ERROR: Unable to create Session ", "2");
      }
      fprintf(stdout, "Successfully imported graph\n");
      // print graph operations:
      // this can be used to identify output and input nodes
      //gris::PrintOp(graph);

      // grab input node operation
      // if the network takes multiple inputs by indexing them from 0 to n inputs and stack them in a vector
      TF_Output input_op = TF_Output{ TF_GraphOperationByName(graph, "input_1"), 0 };
      // Error handling if input is not in graph
      if (input_op.oper == nullptr) {
        TF_DeleteSession(sess, status);
        TF_DeleteStatus(status);
        TF_DeleteGraph(graph);
        throw MUK_EXCEPTION("ERROR: Can't init input_op", "3");
      }

      // Grab output node operation 
      // if the network gives multiple outputs by indexing them from 0 to n outputs and stack them in a vector
      //TF_Output out_op = { TF_GraphOperationByName(graph, "conv2d_27/truediv"), 0 };
      //TF_Output out_op = { TF_GraphOperationByName(graph, "input_1"), 0 };
      TF_Output out_op = TF_Output{ TF_GraphOperationByName(graph, outputOP.c_str()), 0 };
      // Error handling if output is not in graph
      if (out_op.oper == nullptr) {
        TF_DeleteSession(sess, status);
        TF_DeleteStatus(status);
        TF_DeleteGraph(graph);
        throw MUK_EXCEPTION("ERROR: Can't init out_op", "4");
      }
      // Tensor containing the output after forward pass
      TF_Tensor* output_tensor = nullptr;
      // allocate outputImage 
      ImageInt3D::RegionType outputRegion = itkImage.GetLargestPossibleRegion();

      //outputRegion.SetSize(outputSize);
      itk::SmartPointer<ImageInt3D> itkImageOut = ImageInt3D::New();
      itkImageOut->SetRegions(outputRegion);
      itkImageOut->Allocate();

      const int num_dims = TF_GraphGetTensorNumDims(graph, out_op, status);
      std::vector<std::int64_t> dims(num_dims);
      TF_GraphGetTensorShape(graph, out_op, dims.data(), num_dims, status);
      if (TF_GetCode(status) != TF_OK) {
        TF_DeleteGraph(graph);
        TF_DeleteTensor(output_tensor);
        TF_DeleteStatus(status);
        throw MUK_EXCEPTION("ERROR: could not get number of Labels", "5");
      }

      TF_Tensor* tensor;
      // Set number of Labels equal to last index of output tensor assuming[?,H,W,C] structur of softmax layer
      auto numOfLabels = dims[num_dims - 1];
      for(auto b: batches){
        // next batch
        tensor = gris::makeBatch(itkImage, b.start, b.end);
        // Run a Session 
        TF_SessionRun(sess,
          nullptr, // Run options.
          &input_op, &tensor, 1, // Input tensors, input tensor values, number of inputs.
          &out_op, &output_tensor, 1, // Output tensors, output tensor values, number of outputs.
          nullptr, 0, // Target operations, number of targets.
          nullptr, // Run metadata.
          status // Output status.
        );

        // Error handling for Run Session
        if (TF_GetCode(status) != TF_OK) {
          TF_DeleteStatus(status);
          TF_DeleteGraph(graph);
          TF_DeleteTensor(tensor);
          TF_DeleteTensor(output_tensor);
          TF_DeleteStatus(status);
          throw MUK_EXCEPTION("ERROR: run session", "6");
        }

        // convert TF_Tensor to float* 
        const auto output_data = static_cast<float*>(TF_TensorData(output_tensor));

        // insert batch into output image
        itkImageOut = gris::batchtoLabelImage(output_data, numOfLabels, *itkImageOut, b.start, b.end);

        // reset output tensor
        output_tensor = nullptr;
      }

      // Error handling for Run Session
      if (TF_GetCode(status) != TF_OK) {
        TF_DeleteGraph(graph);
        TF_DeleteTensor(tensor);
        TF_DeleteTensor(output_tensor);
        TF_DeleteStatus(status);
        throw MUK_EXCEPTION("ERROR: run session", "7");
      }
      // Error handling for Close Session
      TF_CloseSession(sess, status);
      if (TF_GetCode(status) != TF_OK) {
        TF_DeleteGraph(graph);
        TF_DeleteTensor(tensor);
        TF_DeleteTensor(output_tensor);
        TF_DeleteStatus(status);
        throw MUK_EXCEPTION("ERROR: close session", "8");
      }
      // Error handling for Delete Session
      TF_DeleteSession(sess, status);
      if (TF_GetCode(status) != TF_OK) {
        TF_DeleteStatus(status);
        TF_DeleteGraph(graph);
        TF_DeleteTensor(tensor);
        TF_DeleteTensor(output_tensor);
        throw MUK_EXCEPTION("ERROR: delete session", "9");
      }
      for (auto b : batches)
        std::cout << b.start << ", " << b.end << std::endl;
      // Clean up                                                                      
      TF_DeleteGraph(graph);
      TF_DeleteTensor(tensor);
      TF_DeleteTensor(output_tensor);
      TF_DeleteStatus(status);
      return transposeImage(*itkImageOut);
    }
    catch (std::exception& e)
    {
      LOG_LINE << "EXCEPTION: " << e.what();
      throw e;
    }
  }

  /** \brief converts TF_Datatype to string
  */
  const char* TFDataTypeToString(TF_DataType data_type) 
  {
    switch (data_type) {
    case TF_FLOAT:
      return "TF_FLOAT";
    case TF_DOUBLE:
      return "TF_DOUBLE";
    case TF_INT32:
      return "TF_INT32";
    case TF_UINT8:
      return "TF_UINT8";
    case TF_INT16:
      return "TF_INT16";
    case TF_INT8:
      return "TF_INT8";
    case TF_STRING:
      return "TF_STRING";
    case TF_COMPLEX64:
      return "TF_COMPLEX64";
    case TF_INT64:
      return "TF_INT64";
    case TF_BOOL:
      return "TF_BOOL";
    case TF_QINT8:
      return "TF_QINT8";
    case TF_QUINT8:
      return "TF_QUINT8";
    case TF_QINT32:
      return "TF_QINT32";
    case TF_BFLOAT16:
      return "TF_BFLOAT16";
    case TF_QINT16:
      return "TF_QINT16";
    case TF_QUINT16:
      return "TF_QUINT16";
    case TF_UINT16:
      return "TF_UINT16";
    case TF_COMPLEX128:
      return "TF_COMPLEX128";
    case TF_HALF:
      return "TF_HALF";
    case TF_RESOURCE:
      return "TF_RESOURCE";
    case TF_VARIANT:
      return "TF_VARIANT";
    case TF_UINT32:
      return "TF_UINT32";
    case TF_UINT64:
      return "TF_UINT64";
    default:
      return "Unknown";
    }
  }

  /** \brief prints inputs of operation
  */
  void PrintOpInputs(TF_Graph*, TF_Operation* op) 
  {
    const int num_inputs = TF_OperationNumInputs(op);
    LOG_LINE << "Number inputs: " << num_inputs;
    for (int i = 0; i < num_inputs; ++i) {
      const TF_Input    input = { op, i };
      const TF_DataType type  = TF_OperationInputType(input);
      LOG_LINE << std::to_string(i) << " type : " << TFDataTypeToString(type);
    }
  }

  /** \brief prints shape of all Outputs of operation
  */
  void PrintOpOutputs(TF_Graph* graph, TF_Operation* op)
  {
    TF_Status* status     = TF_NewStatus();
    const int num_outputs = TF_OperationNumOutputs(op);

    LOG_LINE << "Number outputs: " << num_outputs;

    for (int i = 0; i < num_outputs; ++i) {
      const TF_Output output = { op, i };
      const TF_DataType type = TF_OperationOutputType(output);

      LOG_LINE << std::to_string(i) << " type : " << TFDataTypeToString(type);

      const int num_dims = TF_GraphGetTensorNumDims(graph, output, status);
      if (TF_GetCode(status) != TF_OK) {
        LOG_LINE << "Can't get tensor dimensionality";
        continue;
      }

      LOG_LINE << " dims: " << num_dims;
      if (num_dims <= 0) {
        LOG_LINE << " []";;
        continue;
      }

      std::vector<std::int64_t> dims(num_dims);
      TF_GraphGetTensorShape(graph, output, dims.data(), num_dims, status);

      if (TF_GetCode(status) != TF_OK) {
        LOG_LINE << "Can't get get tensor shape";
        continue;
      }

      LOG_LINE << " [";
      for (int j = 0; j < num_dims; ++j) {
        LOG_LINE << dims[j];
        if (j < num_dims - 1) {
          LOG_LINE << ",";
        }
      }
      LOG_LINE << "]";
    }

    TF_DeleteStatus(status);
  }

  /** \brief prints operations of graph
  */
  void PrintOp(TF_Graph* graph) 
  {
    TF_Operation* op;
    TF_Status* status = TF_NewStatus();
    std::size_t pos = 0;
    // iterate trough graph and print out operations
    while ((op = TF_GraphNextOperation(graph, &pos)) != nullptr) {
      // gather attributs of Operations
      const char* name = TF_OperationName(op);
      const char* type = TF_OperationOpType(op);
      const char* device = TF_OperationDevice(op);
      const int num_outputs = TF_OperationNumOutputs(op);
      const int num_inputs = TF_OperationNumInputs(op);

      status = TF_NewStatus();
      const TF_Output output = { op, 0 };
      const int num_dims = TF_GraphGetTensorNumDims(graph, output, status);
      std::string test = name;

      // limit number of outprints to maxmum and/or have certain name (eg. "truediv" = Softmax output) 
      if (pos <= 510 /* && test.find("truediv") != std::string::npos*/)
      {
        std::vector<std::int64_t> dims(num_dims);
        TF_GraphGetTensorShape(graph, output, dims.data(), num_dims, status);

        if (TF_GetCode(status) != TF_OK) {
          LOG_LINE << "Can't get get tensor shape";
          continue;
        }
        // for finding only image outputs
        //if (dims[1] == 512 || dims[2] == 512) {
        // output shape
        LOG << " [";
        for (int j = 0; j < num_dims; ++j) {
          LOG << dims[j];
          if (j < num_dims - 1) {
            LOG << ",";
          }
        }
        LOG << "] ";
        LOG_LINE << pos << ": " << name << " type: " << type << " inputs: " << num_inputs << " outputs: " << num_outputs;
        //} // for finding only image outputs
        //PrintOpInputs(graph, op);
        //PrintOpOutputs(graph, op);
      }
    }
  }

}