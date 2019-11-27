#pragma once
#include "MukImaging/MukImage.h"
#include "MukImaging/muk_imaging_tools.h"

#pragma warning (push)
#pragma warning (disable:4190) // TF_NewWhile' has C-linkage specified, but returns UDT 'TF_WhileParams' which is incompatible with C 
extern "C"
{
#include "tensorflow/c_api.h"
}
#pragma warning (pop)

#include <itkImage.h>

namespace gris
{
  /** \brief Function to read pb Files
  * caller receives ownership of TF_Buffer
  */
  TF_Buffer* read_file(const char* file);

  /** \brief cpp deallocator for TF_Buffer
  */
  void free_buffer(void* data, size_t length);

  /** \brief cpp deallocator for TF_Tensor
  */
  static void DeallocateTensor(void* data, std::size_t, void*);
  
  /** \brief loads ImageInt3D from file
  */
  itk::SmartPointer<gris::muk::ImageInt3D> loadImage(const char* file);

  /** \brief Transposes Image 
  * used to convert itk indexing(HWC) between Tensorflow indexing(CHW)
  */
  itk::SmartPointer<gris::muk::ImageInt3D> transposeImage(const gris::muk::ImageInt3D &itkImage);

  /** \brief Creates Batch input of 2D images from an ImageInt3D image
  * caller is expect to define batch with start and end index in Z dimension
  * caller receives ownership of TF_Tensor*
  */
  TF_Tensor* makeBatch(const gris::muk::ImageInt3D &itkImage, int start, int end);

  /** \brief Imports Graph from file
  * caller receives ownership of TF_Graph* and is expect to do error handling of status
  */
  TF_Graph* importFrozenGraph(const char* file, TF_Status* status);

  /** \brief Creates TF_Session* with Option:allow_growth = True
  * caller receives ownership of TF_Session*
  */
  TF_Session* createSession(TF_Graph* graph, TF_Status* status);

  /** \brief outputs the argmax(label) of pixel
  */
  int labelPixel(const float* data, const itk::Index<3> &idx, int &numOfLabels, int &slice_idx, const itk::Size<3> &outputSize);

  /** \brief writes Image to filepath as out[_number].mhd
  * default is out.mhd
  */
  void writeImage(const gris::muk::ImageInt3D &itkImage, const std::string &filepath, int number);

  /** \brief loads batch data into ImageInt3D
  * expects a softmax layer as output node
  */
  itk::SmartPointer<gris::muk::ImageInt3D> batchtoLabelImage(const float* data, int numOfLabels, const gris::muk::ImageInt3D &itkImage, int &start, int &end);

  /** \brief writes ImageInt3D images with all their channels to filepath
  */
  void batchtoImages(float* &data, const itk::Size<3> &outputSize, int &start, int &end, const std::string &filepath);

  /** \brief predicts Segmentation with tensorflow graph
  */
  itk::SmartPointer<gris::muk::ImageInt3D> predict(const gris::muk::ImageInt3D &itkImage, const std::string &graphFilename, const std::string &outputOp, const int &batchSize);

  /** \brief converts TF_Datatype to string
  */
  const char* TFDataTypeToString(TF_DataType data_type);

  /** \brief prints inputs of operation
  */
  void PrintOpInputs(TF_Graph*, TF_Operation* op);

  /** \brief prints shape of all Outputs of operation
  */
  void PrintOpOutputs(TF_Graph* graph, TF_Operation* op);

  /** \brief prints operations of graph
  */
  void PrintOp(TF_Graph* graph);
}