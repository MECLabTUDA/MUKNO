#include "private/muk.pch"
#include "MultiSceneSinglePathQuery.h"
#include "statistics.h"

#include "MukCommon/MukException.h"
#include "MukCommon/EvaluationFactory.h"
#include "MukCommon/MukScene.h"
#include "MukCommon/PathCollection.h"
#include "MukCommon/mukIO.h"

#include "MukQt/MultipleDoubleVectorPlot.h"
#include "MukQt/BoxPlots.h"
#include "MukQt/EvaluationWindow.h"

#include <gstd/XmlNode.h>
#include <gstd/XmlDocument.h>

#include <boost/format.hpp>
#include <boost/filesystem.hpp>

#include<iostream>
#include<fstream>
#include<iterator>
#include<vector>

#ifdef max
#undef max
#endif

namespace
{
  namespace fs = boost::filesystem;
  using namespace gris::muk;
  void getlineThrows(ifstream& ifs, std::string& line)
  {
    if ( ! std::getline(ifs, line))
    {
      MUK_EXCEPTION_SIMPLE("Invalid MukEvaluation-File");
    }
  }

  std::string print(const std::vector<double>& data)
  {
    std::stringstream ss;
    std::for_each(data.begin(), data.end(), [&] (const double& d) { ss << d << " "; });
    return ss.str();
  }  
}

namespace gris
{
namespace muk
{
  REGISTER_EVALUATION(MultiSceneSinglePathQuery);

  /**
  */
  MultiSceneSinglePathQuery::MultiSceneSinglePathQuery()
  {
  }

  /**
  */
  MultiSceneSinglePathQuery::~MultiSceneSinglePathQuery()
  {
  }

  /**
  */
  void MultiSceneSinglePathQuery::read(const std::string& xml)
  {
    mInput.pathName.clear();
    mInput.sceneNames.clear();    
    auto pDoc = XmlDocument::create("tmp");
    pDoc->fromString(xml.c_str());
    auto root = pDoc->getRoot();
    auto node = root.getChild("scenes");
    std::vector<gris::XmlNode> nodes = node.getChildren();
    for (const auto& subNode : nodes)
    {      
      mInput.sceneNames.push_back(subNode.getValue());
    }
    node = root.getChild("path");
    mInput.pathName = node.getValue();
    
    LOG << "evaluate path '" << mInput.pathName << "'\n";
    LOG << "in scenes:\n";
    for(const auto& str : mInput.sceneNames)
      LOG << "'" << str << "'\n";
    LOG_LINE << "";
  }
  
  /**

    expects:
    <IMukEvaluation>
      <implementation> MultiSceneSinglePathQuery< /implementation>
      <scenes>
        <scene> filename  </scene>
        <scene> filename  </scene>
        <scene> filename  </scene>
        ...
      </scenes>
      <path>       name      </path>
      </IMukEvaluation>
  */
  void MultiSceneSinglePathQuery::evaluate()
  {
    read(mQuery);
    validateQueryInput();
    mOutput.clear();
    LOG_LINE << "confirmed: input valid";
    auto pScene = std::make_unique<MukScene>();
    for (const auto& str : mInput.sceneNames)
    {      
      pScene->load(str);
      pScene->setName(str);
      readSingleScene(pScene.get());
      LOG_LINE << " evaluated scene";
    }
  }

  /**
  */
  bool MultiSceneSinglePathQuery::validateQueryInput()
  {
    for (const auto& str : mInput.sceneNames)
    {
      if ( ! fs::is_regular_file(str.c_str()) )
        throw MUK_EXCEPTION("scene-file does not exist!", str.c_str());
      auto doc = gris::XmlDocument::read(str.c_str());
      auto vecPaths = doc->getRoot().getChild("PathCollections").getChildren();      
      auto iter = std::find_if(vecPaths.begin(), vecPaths.end(), 
         [&] (const auto& node) { return 0 == mInput.pathName.compare(node.getChild("Name").getValue()); });;
      if (iter==vecPaths.end())
      {
        const std::string msg = (boost::format("Scene %s does not have the following path") % str).str();
        throw MUK_EXCEPTION(msg.c_str(), mInput.pathName.c_str());
      } 
    }
    return true;
  }

  /**
  */
  void MultiSceneSinglePathQuery::readSingleScene(MukScene* pScene)
  {
    mOutput.push_back(QueryOutput());
    auto& result = mOutput.back();
    auto& collection = pScene->getPathCollection(mInput.pathName);
    if (collection.getPaths().empty())
      throw MUK_EXCEPTION("path collection is empty in scene:", pScene->getName().c_str());    
    result.distances = computeDistances(*pScene->getCollisionDetector(), collection.getPaths()[0].getPath(), collection.getProblemDefinition()->getRadius());
    
    result.sceneName = pScene->getName();
    LOG << "scene: " << pScene->getName() << "\n";

    auto minDist = result.distances.empty() ? std::numeric_limits<double>::max() : *std::min_element(result.distances.begin(), result.distances.end());
    LOG << " N dist " << result.distances.size() << "\n";
    LOG << " minDist " << minDist << "\n";

    result.curvatures = computeCurvatures(collection.getPaths()[0].getPath());
    auto minCurv = result.curvatures.empty() ? 0 : *std::min_element(result.curvatures.begin(), result.curvatures.end());
    LOG << " N curv " << result.curvatures.size() << "\n";
    LOG << " minCurv " << minCurv << "\n";
  }

  /**
  */
  const std::vector<MultiSceneSinglePathQuery::QueryOutput>* MultiSceneSinglePathQuery::getOutput() const
  {
    return &mOutput;
  }

  /**
  */
  void MultiSceneSinglePathQuery::saveResults(const char* filename) const
  {    
    //fs::path fn (filename);
    std::ofstream ofs (filename);
    ofs << name() << std::endl;
    ofs << mInput.pathName << std::endl;
    ofs << mInput.sceneNames.size() << std::endl;
    for (const auto& scene : mOutput)
    {
      ofs << scene.sceneName << std::endl;      
      ofs << scene.distances.size() << " " << ::print(scene.distances) << std::endl;      
      ofs << scene.curvatures.size() << " " << ::print(scene.curvatures) << std::endl;
    }
    LOG_LINE << "saved to " << filename;
  }

  /**
  */  
  void MultiSceneSinglePathQuery::loadResults(const char* filename)
  {
    //fs::path fn (filename);
    std::ifstream ifs (filename);
    std::string line;
    getlineThrows(ifs ,line);
    getlineThrows(ifs ,line);
    getlineThrows(ifs ,line);
    const size_t N = atoi(line.c_str());
    mOutput.resize(N);
    for (size_t i(0); i<N; ++i)
    {
      getlineThrows(ifs, mOutput[i].sceneName);
      getlineThrows(ifs, line);
      {
        std::stringstream ss(line);
        size_t M;
        ss >> M;
        mOutput[i].distances.resize(M);
        for (size_t j(0); j<M; ++j)
        {
          ss >> mOutput[i].distances[j];
        }
      }
      getlineThrows(ifs, line);
      {
        std::stringstream ss(line);
        size_t M;
        ss >> M;
        mOutput[i].curvatures.resize(M);
        for (size_t j(0); j<M; ++j)
        {
          ss >> mOutput[i].curvatures[j];
        }
      }
    }
    LOG << "loaded " << filename << "\n";
    LOG << "   Scenes: " << mOutput.size() << "\n";
    for (size_t i(0); i<mOutput.size(); ++i)
    {
      LOG << "   name: " << mOutput[i].sceneName << "\n";
      LOG << "   numel distances " << mOutput[i].distances.size() << "\n";
      LOG << "   numel curvatures " << mOutput[i].curvatures.size() << "\n";
    }
      
    LOG_LINE << "";
  }

  void MultiSceneSinglePathQuery::visualize(EvaluationWindow* pWindow)
  {
    // create 4 plots
    // first column
    auto pPlot = std::make_unique<MultipleDoubleVectorPlot>();
    {
      pPlot->setTitle("Distances");
      pPlot->setXLabel("path index");
      pPlot->setYLabel("distance to risk structures (mm)");
      const size_t N = mOutput.size();
      pPlot->setNumberOfGraphs(N);
      for (size_t i(0); i<N; ++i)
      {
        pPlot->setData(i, mOutput[i].distances);
        fs::path p(mOutput[i].sceneName.c_str());
        pPlot->setGraphTitle(i, p.parent_path().stem().string().c_str());
      }
      pPlot->adjust();
      pWindow->addWidget(pPlot.release(), 0, 0, 1, 1);
    }
    pPlot = std::make_unique<MultipleDoubleVectorPlot>();
    {
      pPlot->setTitle("Curvatures");
      pPlot->setXLabel("path index");
      pPlot->setYLabel("curvature");
      const size_t N = mOutput.size();
      pPlot->setNumberOfGraphs(N);
      for (size_t i(0); i<N; ++i)
      {
        pPlot->setData(i, mOutput[i].curvatures);
        fs::path p(mOutput[i].sceneName.c_str());
        pPlot->setGraphTitle(i, p.parent_path().stem().string().c_str());
      }
      pWindow->addWidget(pPlot.get(), 1, 0, 1, 1);
      pPlot->adjust();
      pPlot.release();
    }
    // second column
    auto pBoxPlot = std::make_unique<BoxPlots>();
    {
      pBoxPlot->setTitle("Distances");      
      pBoxPlot->setYLabel("distance to risk structures (mm)");
      const size_t N = mOutput.size();
      for (size_t i(0); i<N; ++i)
      {
        pBoxPlot->addBox(mOutput[i].distances);
        fs::path p(mOutput[i].sceneName.c_str());
        pBoxPlot->setSampleName(i, p.parent_path().stem().string().c_str());
      }
      pBoxPlot->adjust();
      pWindow->addWidget(pBoxPlot.release(), 0, 1, 1, 1);
    }
    pBoxPlot = std::make_unique<BoxPlots>();
    {
      pBoxPlot->setTitle("Curvatures");      
      pBoxPlot->setYLabel("curvature along the path");
      const size_t N = mOutput.size();
      for (size_t i(0); i<N; ++i)
      {
        pBoxPlot->addBox(mOutput[i].curvatures);
        fs::path p(mOutput[i].sceneName.c_str());
        pBoxPlot->setSampleName(i, p.parent_path().stem().string().c_str());
      }      
      pBoxPlot->adjust();
      pWindow->addWidget(pBoxPlot.release(), 1, 1, 1, 1);
    }
    // third column    
    pBoxPlot = std::make_unique<BoxPlots>();
    {
      pBoxPlot->setTitle("Distances");      
      pBoxPlot->setYLabel("distance to risk structures (mm)");
      const size_t N = mOutput.size();
      std::vector<double> overallData;
      for (size_t i(0); i<N; ++i)
      {
        std::copy(mOutput[i].distances.begin(), mOutput[i].distances.end(), back_inserter(overallData));
      }
      pBoxPlot->addBox(overallData);
      pBoxPlot->setSampleName(0, "Combined");
      pBoxPlot->adjust();
      pWindow->addWidget(pBoxPlot.release(), 0, 2, 1, 1);
    }
    pBoxPlot = std::make_unique<BoxPlots>();
    {
      pBoxPlot->setTitle("Curvatures");      
      pBoxPlot->setYLabel("curvature along the path");
      const size_t N = mOutput.size();
      std::vector<double> overallData;
      for (size_t i(0); i<N; ++i)
      {
        std::copy(mOutput[i].curvatures.begin(), mOutput[i].curvatures.end(), back_inserter(overallData));
      }
      pBoxPlot->addBox(overallData);
      pBoxPlot->setSampleName(0, "Combined");
      pBoxPlot->adjust();
      pWindow->addWidget(pBoxPlot.release(), 1, 2, 1, 1);
    }
  }
}
}