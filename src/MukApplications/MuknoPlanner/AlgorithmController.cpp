#include "private/muk.pch"
#include "AlgorithmController.h"
#include "VisualizationController.h"
#include "private/PropertyDetails.h"

#include "AppControllers.h"
#include "PropertyController.h"

#include "MukCommon/vtk_tools.h"

#include "MukAppModels/AlgorithmModel.h"
#include "MukAppModels/AppModels.h"
#include "MukAppModels/ApplicationModel.h"
#include "MukAppModels/VisualizationModel.h"
#include "MukAppModels/WorldVisualizationModel.h"

#include "MukImaging/muk_imaging_tools.h"

#include "MukVisualization/VisAbstractObject.h"

#include "MukQt/AlgConnection.h"
#include "MukQt/AlgGraphicsView.h"
#include "MukQt/AlgItem.h"
#include "MukQt/AlgPort.h"
#include "MukQt/muk_qt_tools.h"
#include "MukQt/MuknoPlannerMainWindow.h"
#include "MukQt/TabImaging.h"

#include <gstd/XmlDocument.h>
#include <gstd/XmlNode.h>
#include <gstd/XmlAttribute.h>

#include <itkImageFileWriter.h>
#include <itkImagetoVTKImageFilter.h>
#include <itkImageRegionConstIterator.h>

#include <vtkImageData.h>
#include <vtkLine.h>
#include <vtkPolyData.h>

#include <qgraphicsview.h>
#include <qtreewidget.h>

#include <boost/filesystem.hpp>

namespace
{
  std::string getAlgVisName(const std::string& algoName, int algoId)
  {
    return algoName + "_" + std::to_string(algoId);
  }
}

namespace gris
{
namespace muk
{
  /**
  */
  AlgorithmController::AlgorithmController()
    : BaseController()
  {
  }

  /**
  */
  void AlgorithmController::setupConnections()
  {
    connect(mpTabImaging, &TabImaging::algorithmDeleted,  this, &AlgorithmController::deleteAlgorithm);
    connect(mpTabImaging, &TabImaging::connectionAdded,   this, &AlgorithmController::addConnection);
    connect(mpTabImaging, &TabImaging::connectionDeleted, this, &AlgorithmController::deleteConnection);
    connect(mpTabImaging, &TabImaging::showAlgProperties, this, &AlgorithmController::showAlgProperties);
    connect(mpTabImaging, &TabImaging::showOutput,        this, &AlgorithmController::showOutput);
    connect(mpTabImaging, &TabImaging::execute,           this, &AlgorithmController::executePipeline);
    connect(mpTabImaging, &TabImaging::clearSceneClicked, this, &AlgorithmController::newAlgorithm);
    connect(mpTabImaging, &TabImaging::saveAlgorithmClicked, this, &AlgorithmController::saveAlgorithm);
    connect(mpTabImaging, &TabImaging::loadAlgorithmClicked, [&](const std::string& str) { this->loadAlgorithm(str, false); });
    connect(mpTabImaging, &TabImaging::textDroppedOnAlgorithm, this, &AlgorithmController::setAlgorithmPropFromDrop);
    connect(mpTabImaging, &TabImaging::textDroppedOnView, this, &AlgorithmController::createAlgorithmFromDrop);
  }

  /** \brief initialize TabImaging
  */
  void AlgorithmController::initTabImaging()
  {
    mpTabImaging->clear();
    auto pListItems = mpModels->pAlgorithmModel->listItems();
    mpTabImaging->setAlgorithmList(pListItems);
  }

  /**
  */
  void AlgorithmController::newAlgorithm()
  {
    // clear model
    mpModels->pAlgorithmModel->clear();
    // clear visualization
    mpTabImaging->clearAlgorithms();
  }

  /** \brief algorithm added to scene. Add algorithm to AlgorithmManager
  */
  void AlgorithmController::addAlgorithm(const std::string& name, const QPointF& position)
  {
    auto idAlgo   = mpModels->pAlgorithmModel->addAlgorithm(name);
    auto inPorts  = mpModels->pAlgorithmModel->getAlgorithm(idAlgo).sizeInputs();
    auto outPorts = mpModels->pAlgorithmModel->getAlgorithm(idAlgo).sizeOutputs();
    mpTabImaging->addAlgItem(idAlgo, name, "", inPorts, outPorts, position);
  }

  /** \brief algorithm deleted. Delete algorithm from AlgorithmManager
  */
  void AlgorithmController::deleteAlgorithm(unsigned int id)
  {
    auto name = mpModels->pAlgorithmModel->getAlgorithm(id).name();
    mpModels->pWorldVisModel->deleteSegmentationObject(getAlgVisName(name, id));
    mpModels->pAlgorithmModel->deleteAlgorithm(id);
  }

  /** \brief connection between two algorithms added in the scene. Add connection in AlgorithmManager
  */
  void AlgorithmController::addConnection(AlgPort *p1, AlgPort *p2)
  {
    unsigned int srcID, srcPort, dstID, dstPort;
    if (p1->isOutput()) 
    {
      // Source
      srcID = p1->getRefID();
      srcPort = p1->getPortID();
      // Destination target 
      dstID = p2->getRefID();
      dstPort = p2->getPortID();
    }
    else
    {
      // Destination target 
      dstID = p1->getRefID();
      dstPort = p1->getPortID();
      // Source
      srcID = p2->getRefID();
      srcPort = p2->getPortID();
    }
    mpModels->pAlgorithmModel->addConnection(srcID, srcPort, dstID, dstPort);
  }

  /** \brief connection between two algortihms deleted. Delete connection in AlgortihmManager
  */
  void AlgorithmController::deleteConnection(AlgPort *p1, AlgPort *p2)
  {
    unsigned int srcID, srcPort, dstID, dstPort;
    if (p1->isOutput())
    {
      // Source
      srcID = p1->getRefID();
      srcPort = p1->getPortID();
      // Destination target 
      dstID = p2->getRefID();
      dstPort = p2->getPortID();
    }
    else
    {
      // Destination target 
      dstID = p1->getRefID();
      dstPort = p1->getPortID();
      // Source
      srcID = p2->getRefID();
      srcPort = p2->getPortID();
    }
    mpModels->pAlgorithmModel->deleteConnection(srcID, srcPort, dstID, dstPort);
  }

  /** \brief show the properties of the selected algorithm
  */
  void AlgorithmController::showAlgProperties(unsigned int id)
  {
    gstd::DynamicProperty* pObj = &mpModels->pAlgorithmModel->getAlgorithm(id);
    mpControls->mpPropControl->showProperty(EnPropertyType::enAlgorithm, pObj);
  }

  /** \brief show the output of the selected port
  */
  void AlgorithmController::showOutput(unsigned int algoId, bool isOutput, unsigned int portId) 
  {
    if ( ! isOutput)
    {
      // compute, were the original image came from
      auto ids = mpModels->pAlgorithmModel->getSource(std::make_pair(algoId, portId));
      if (ids.first == algoId)
      {
        LOG_LINE << "no algorithm connected";
        return;
      }
      algoId = ids.first;
      portId = ids.second;
    }

    auto& alg        = mpModels->pAlgorithmModel->getAlgorithm(algoId);
    auto displayType = alg.getDisplayType();
    auto dataType    = alg.getOutputType(portId);

    switch (dataType)
    {
      case enImageInt2D:
      {
        LOG_LINE << "show 2D image";
        vtkSmartPointer<vtkImageData> pImage;
        ImageInt2D::Pointer img = static_cast<ImageInt2D*>(alg.getOutput(portId));
        auto trafo = make_itk<itk::ImageToVTKImageFilter<ImageInt2D>>();
        trafo->SetInput(img);
        trafo->Update();
        pImage = trafo->GetOutput();
        if (displayType == EnDisplayType::enDisplayImage2D)
          mpModels->pWorldVisModel->setCtImage(pImage);
        else
          mpModels->pWorldVisModel->setSegmentationImage(pImage);
        break;
      }
      case enImageInt3D:
      {
        vtkSmartPointer<vtkImageData> pImage;
        ImageInt3D::Pointer mukImg = static_cast<ImageInt3D*>(alg.getOutput(portId));
        auto trafo = make_itk<itk::ImageToVTKImageFilter<ImageInt3D>>();
        trafo->SetInput(mukImg);
        trafo->Update();
        pImage = trafo->GetOutput();
        if (displayType == EnDisplayType::enDisplayImage3D)
          mpModels->pWorldVisModel->setCtImage(pImage);
        else
          mpModels->pWorldVisModel->setSegmentationImage(pImage);
        break;
      }
      case enImageFloat3D:
      {
        vtkSmartPointer<vtkImageData> pImage;
        ImageFloat3D::Pointer mukImg = static_cast<ImageFloat3D*>(alg.getOutput(portId));
        auto trafo = make_itk<itk::ImageToVTKImageFilter<ImageFloat3D>>();
        trafo->SetInput(mukImg);
        trafo->Update();
        pImage = trafo->GetOutput();
        if (displayType == EnDisplayType::enDisplayImage3D)
          mpModels->pWorldVisModel->setCtImage(pImage);
        else
          mpModels->pWorldVisModel->setSegmentationImage(pImage);
        break;
      }
      case enVtkPolyData:
      {
        // load polydata output
        void* algData = alg.getOutput(portId);
        auto pData = make_vtk<vtkPolyData>();
        pData->DeepCopy(static_cast<vtkPolyData*>(algData));
        auto name = getAlgVisName(std::string(mpModels->pAlgorithmModel->getAlgorithm(algoId).name()), algoId);
        mpModels->pWorldVisModel->setSegmentationObject(name, pData);
        mpControls->mpPropControl->addAlgorithmOutput(name);
      }
      case enVtkImage:
      {
        // ToDo
        break;
      }
      case enVtkMesh:
      {
        // ToDo
        break;
      }
      case enGradientImage3D:
      {
        auto poly   = make_vtk<vtkPolyData>();
        auto points = make_vtk<vtkPoints>();
        auto lines  = make_vtk<vtkCellArray>();
        Vec3d center (-50, -140, 304);
        Vec3d limits (5, 5, 5);
        std::vector<Vec3d> roots;
        std::vector<Vec3d> vecs;

        GradientImage3D::Pointer img = static_cast<GradientImage3D*>(alg.getOutput(portId));
        auto iter = itk::ImageRegionConstIterator<GradientImage3D>(img, img->GetBufferedRegion());
        while (!iter.IsAtEnd())
        {
          auto vec   = iter.Get();
          auto index = iter.GetIndex();
          ++iter;;
          itk::Point<double, 3> p;
          img->TransformIndexToPhysicalPoint(index, p);
          // we cannot add the whole image's vector field, limit it to a small part (Todo: add way to specify this)
          auto dist = Vec3d( center[0]-p[0], center[1]-p[1], center[2]-p[2] ).abs();
          if (dist[0] > limits[0] || dist[1] > limits[1] || dist[2] > limits[2])
          {
            continue;
          }
          roots.push_back(Vec3d(p[0], p[1], p[2]));
          vecs. push_back(Vec3d(vec[0], vec[1], vec[2]));
        }

        auto maxIter = std::max_element(vecs.begin(), vecs.end(), [&] (const auto& lhs, const auto& rhs) { return lhs.squaredNorm() < rhs.squaredNorm(); } );
        double maximum(1.0);
        if ( ! vecs.empty())
          maximum = std::max(maximum, maxIter->norm());
        for (size_t i(0); i<roots.size(); ++i)
        {
          vtkIdType id[2];
          id[0] = points->InsertNextPoint(roots[i].data());
          id[1] = points->InsertNextPoint( (roots[i] + vecs[i] / maximum).data() );
          auto line = make_vtk<vtkLine>();
          line->GetPointIds()->SetId(0, id[0]);
          line->GetPointIds()->SetId(1, id[1]);
          lines->InsertNextCell(line);
        }

        poly->SetPoints(points);
        poly->SetLines(lines);
        auto name = getAlgVisName(std::string(mpModels->pAlgorithmModel->getAlgorithm(algoId).name()), algoId);
        mpModels->pWorldVisModel->setSegmentationObject(name, poly);
        mpControls->mpPropControl->addAlgorithmOutput(name);
        break;
      }
    };
    mpModels->pWorldVisModel->render();
    mpModels->pVisModel->render();
  }

  /** \brief execute all algorithms
  */
  void AlgorithmController::executePipeline() 
  {
    LOG_LINE << "processing ...";
    mpModels->pAlgorithmModel->update();
    LOG_LINE << "... finished";
  }

  /** \brief initialize controller 
  */
  void AlgorithmController::initialize()
  {
    mpTabImaging = mpMainWindow->mpTabImaging;
    initTabImaging();
  }

  /** \brief saves the algorithm currently stored in the AlgorithModel
  */
  void AlgorithmController::saveAlgorithm(const std::string& filename) const
  {
    mpModels->pAlgorithmModel->saveAlgorithm(filename);
    // save view
    {
      auto doc = XmlDocument::read(filename.c_str());
      auto root = doc->getRoot().getChild("Algorithm");
      auto ndNodes = root.getChild("AlgNodes").getChildren();
      const auto* scene   = mpTabImaging->getGraphicsScene();
      const auto items    = scene->items();
      for (auto ndAlg : ndNodes)
      {
        const auto id = std::atoi(ndAlg.getChild("ID").getValue());
        // find the corresponding algitem
        auto iterAlgItem = std::find_if(items.begin(), items.end(), [&] (const auto* item)
        {
          auto* algItem = dynamic_cast<const AlgItem*>(item);
          if ( ! algItem)
            return false;
          return algItem->getRefID() == id;
        });
        if (iterAlgItem==items.end())
          continue;
        auto* algItem = dynamic_cast<const AlgItem*>(*iterAlgItem);
        auto ndView = ndAlg.addChild("AlgView");
        auto ndPos  = ndView.addChild("Position");
        ndPos.addChild("X").setValue(std::to_string(algItem->pos().x()).c_str());
        ndPos.addChild("Y").setValue(std::to_string(algItem->pos().y()).c_str());
      }
      XmlDocument::save(filename.c_str(), *doc);
    }
  }

  /** \brief loads a saved algorithm into the AlgorithModel

  also updates the view
  */
  void AlgorithmController::loadAlgorithm(const std::string& filename, bool badPropertiesThrow)
  {
    // load algorithm into model
    mpModels->pAlgorithmModel->loadAlgorithm(filename, badPropertiesThrow);
    // adjust view
    mpTabImaging->clearAlgorithms();
    const QSignalBlocker blocker(mpTabImaging);
    {
      auto doc = XmlDocument::read(filename.c_str());
      auto root = doc->getRoot().getChild("Algorithm");
      auto ndNodes = root.getChild("AlgNodes").getChildren();
      const auto* scene = mpTabImaging->getGraphicsScene();
      for (auto ndAlg : ndNodes)
      {
        const auto id = std::atoi(ndAlg.getChild("ID").getValue());
        const auto& alg = mpModels->pAlgorithmModel->getAlgorithm(id);
        // alg data
        const auto& name   = alg.name();
        const auto& alias  = alg.getAlias();
        const auto sizeIn  = alg.sizeInputs();
        const auto sizeOut = alg.sizeOutputs();
        // view data
        const auto ndView = ndAlg.getChild("AlgView");
        const auto ndPos = ndView.getChild("Position");
        const auto x = std::atof(ndPos.getChild("X").getValue());
        const auto y = std::atof(ndPos.getChild("Y").getValue());
        QPointF pos(x, y);
        mpTabImaging->addAlgItem(id, name, alias, sizeIn, sizeOut, pos);
      }
      auto ndEdges = root.getChild("Edges").getChildren();
      for (const auto& node : ndEdges)
      {
        const auto srcID   = std::atoi(node.getChild("SourceId").getValue());
        const auto srcPort = std::atoi(node.getChild("SourcePortId").getValue());
        const auto dstID   = std::atoi(node.getChild("TargetId").getValue());
        const auto dstPort = std::atoi(node.getChild("TargetPortId").getValue());
        mpTabImaging->addAlgEdge(srcID, srcPort, dstID, dstPort);
      }
    }
  }
  
  /**
  */
  void AlgorithmController::createAlgorithmFromDrop(const std::string& str, const QPointF& pos)
  {
    namespace fs = boost::filesystem;
    LOG_LINE << "dropped " << str;
    if (str.substr(0, 4) != "file")
    {
      //LOG_LINE << "drop supports only files";
      //return;
      addAlgorithm(str, pos);
      return;
    }
    auto fn = fs::path(str.substr(8));
    if ( ! fs::is_regular_file(fn))
    {
      LOG_LINE << "no file: " << fn;
      return;
    }
    enum EnFileType
    {
      enUnknown,
      enMhd,
      enVtk,
      enAlgorithm
    };
    EnFileType type = enUnknown;
    auto ext = fn.extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
    if (ext == ".mhd" || ext == ".mha")
      type = enMhd;
    else if (ext == ".vtk")
      type = enVtk;
    else if (ext == ".alg")
      type = enAlgorithm;
    switch (type)
    {
      case enMhd:
        addAlgorithm("ImageReader", pos);
        break;
      case enVtk:
        addAlgorithm("PolyDataReader", pos);
        break;
      case enAlgorithm:
        loadAlgorithm(fn.string());
        break;
      default:
        throw MUK_EXCEPTION("no file drop available for file type", fn.string().c_str());
    };
    switch (type)
    {
      case enMhd: 
      case enVtk:
      {
        const auto id = mpTabImaging->getAlgItemHandler()->algorithmAt(pos);
        auto& pAlg    = mpModels->pAlgorithmModel->getAlgorithm(id);
        const auto* propName = "FileName";
        const auto* propName2 = "Filename";
        if (pAlg.hasProperty(propName))
          pAlg.setProperty(propName, fn.string());
        else if (pAlg.hasProperty(propName2))
          pAlg.setProperty(propName2, fn.string());
        break;
      }
    };
  }

  /**
  */
  void AlgorithmController::setAlgorithmPropFromDrop(const std::string& str, unsigned int id)
  {
    namespace fs = boost::filesystem;
    if (str.substr(0, 4) != "file")
    {
      LOG_LINE << "drop supports only files";
      return;
    }
    auto filename = str.substr(8);
    if (!fs::is_regular_file(filename))
    {
      LOG_LINE << "no file: " << filename;
      return;
    }
    auto& pAlg    = mpModels->pAlgorithmModel->getAlgorithm(id);
    const auto* propName = "FileName";
    const auto* propName2 = "Filename";
    if (pAlg.hasProperty(propName))
      pAlg.setProperty(propName, filename);
    else if (pAlg.hasProperty(propName2))
      pAlg.setProperty(propName2, filename);
  }
} // namespace muk
} // namespace gris