#include "private/muk.pch"
#include "PathPlanningComparisonQuery.h"
#include "statistics.h"

#include "MukCommon/MukException.h"
#include "MukCommon/EvaluationFactory.h"
#include "MukCommon/MukScene.h"
#include "MukCommon/PathCollection.h"
#include "MukCommon/ICollisionDetector.h"
#include "MukCommon/mukIO.h"

#include "MukQt/MultipleDoubleVectorPlot.h"
#include "MukQt/BoxPlots.h"
#include "MukQt/EvaluationWindow.h"

#include <gstd/XmlNode.h>
#include <gstd/XmlDocument.h>

#include <qcustomplot.h>

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
    REGISTER_EVALUATION(PathPlanningComparisonQuery);

    /**
    */
    PathPlanningComparisonQuery::PathPlanningComparisonQuery()
    {
    }

    /**
    */
    PathPlanningComparisonQuery::~PathPlanningComparisonQuery()
    {
    }

    /**
    */
    void PathPlanningComparisonQuery::read(const std::string& xml)
    {
      mInput.sceneNames.clear();
      mInput.pathNameFirst.clear();
      mInput.pathNameSecond.clear();
      mInput.inactiveObstacles.clear();
      
      auto pDoc = XmlDocument::create("tmp");
      pDoc->fromString(xml.c_str());
      auto root = pDoc->getRoot();
      std::vector<gris::XmlNode> nodes = root.getChild("scenes").getChildren();
      for (const auto& subNode : nodes)
      {      
        mInput.sceneNames.push_back(subNode.getValue());
      }
      auto node = root.getChild("first-path");
      mInput.pathNameFirst = node.getValue();
      node = root.getChild("second-path");
      mInput.pathNameSecond = node.getValue();
      nodes = root.getChild("inactive-obstacles").getChildren();
      for (auto node : nodes)
      {
        mInput.inactiveObstacles.push_back(node.getValue());
      }      

      LOG << "evaluate paths\n   '" << mInput.pathNameFirst << "'\n";
      LOG << "    '" << mInput.pathNameSecond << "'\n";
      LOG << "in scenes:\n";
      for(const auto& str : mInput.sceneNames)
        LOG << "'" << str << "'\n";
      LOG << "with deactivated obstacles:\n";
      for(const auto& str : mInput.inactiveObstacles)
        LOG << "'" << str << "'\n";
      LOG_LINE << "";
    }

    /**

    expects:
    <IMukEvaluation>
    <implementation> PathPlanningComparisonQuery< /implementation>
    <scenes>
    <scene> filename  </scene>
    <scene> filename  </scene>
    ...
    </scenes>
    <active-obstacles>
      <obstacle> </obstacle>
      <obstacle> </obstacle>
      ...
    <active-obstacles>
    <path>       firstPathName      </path>
    <path>       SecondPathName      </path>
    </IMukEvaluation>
    */
    void PathPlanningComparisonQuery::evaluate()
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
    bool PathPlanningComparisonQuery::validateQueryInput()
    {
      for (const auto& str : mInput.sceneNames)
      {
        if ( ! fs::is_regular_file(str.c_str()) )
          throw MUK_EXCEPTION("scene-file does not exist!", str.c_str());
        auto doc = gris::XmlDocument::read(str.c_str());
        auto vecPaths = doc->getRoot().getChild("PathCollections").getChildren();      
        auto iter = std::find_if(vecPaths.begin(), vecPaths.end(), 
          [&] (const auto& node) { return 0 == mInput.pathNameFirst.compare(node.getChild("Name").getValue()); });;
        if (iter==vecPaths.end())
        {
          const std::string msg = (boost::format("Scene %s does not have the following path") % str).str();
          throw MUK_EXCEPTION(msg.c_str(), mInput.pathNameFirst.c_str());
        }
        auto iter2 = std::find_if(vecPaths.begin(), vecPaths.end(), 
          [&] (const auto& node) { return 0 == mInput.pathNameSecond.compare(node.getChild("Name").getValue()); });;
        if (iter2==vecPaths.end())
        {
          const std::string msg = (boost::format("Scene %s does not have the following path") % str).str();
          throw MUK_EXCEPTION(msg.c_str(), mInput.pathNameSecond.c_str());
        }
      }
      return true;
    }

    /**
    */
    void PathPlanningComparisonQuery::readSingleScene(MukScene* pScene)
    {
      // obstacles
      {
        auto keys = pScene->getObstacleKeys();
        for(auto& key : keys)
          pScene->getCollisionDetector()->setActive(key, true);
        for(auto& key : mInput.inactiveObstacles)
          pScene->getCollisionDetector()->setActive(key, false);
        pScene->getCollisionDetector()->rebuild();
      }
      mOutput.push_back(QueryOutput());
      auto& result = mOutput.back();
      result.pathNameFirst = mInput.pathNameFirst;
      result.pathNameSecond = mInput.pathNameSecond;
      result.sceneName = pScene->getName();
      auto& coll1 = pScene->getPathCollection(mInput.pathNameFirst);
      auto& prob1 = *coll1.getProblemDefinition();
      auto& coll2 = pScene->getPathCollection(mInput.pathNameSecond);
      auto& prob2 = *coll1.getProblemDefinition();
      if (prob1.getSafetyDist() != prob2.getSafetyDist() || prob1.getKappa() != prob2.getKappa())
      {
        throw MUK_EXCEPTION("path configuration (kappa or safety distance) for path 1 is different from path 2 in scene:", pScene->getName().c_str());
      }
      result.maxDistance  = prob1.getSafetyDist();
      result.maxCurvature = prob1.getKappa();      
      {        
        if (coll1.getPaths().empty())
          throw MUK_EXCEPTION("path collection is empty in scene:", pScene->getName().c_str());    
        result.distancesFirst  = computeDistances(*pScene->getCollisionDetector(), coll1.getPaths()[0].getPath(), prob1.getRadius());
        result.curvaturesFirst = computeCurvatures(coll1.getPaths()[0].getPath());
        result.maxDistance  = prob1.getSafetyDist();
        result.maxCurvature = prob1.getKappa();
      } 
      {        
        if (coll2.getPaths().empty())
          throw MUK_EXCEPTION("path collection is empty in scene:", pScene->getName().c_str());    
        result.distancesSecond = computeDistances(*pScene->getCollisionDetector(), coll2.getPaths()[0].getPath(), prob2.getRadius());
        result.curvaturesSecond = computeCurvatures(coll2.getPaths()[0].getPath());
      }
    }

    /**
    */
    const std::vector<PathPlanningComparisonQuery::QueryOutput>* PathPlanningComparisonQuery::getOutput() const
    {
      return &mOutput;
    }

    /**
    */
    void PathPlanningComparisonQuery::saveResults(const char* filename) const
    {    
      std::ofstream ofs (filename);
      ofs << name() << std::endl;
      ofs << mInput.pathNameFirst << std::endl;
      ofs << mInput.pathNameSecond << std::endl;
      ofs << mInput.sceneNames.size() << std::endl;
      for (const auto& queryOutput : mOutput)
      {
        ofs << queryOutput.sceneName << std::endl;
        ofs << queryOutput.maxDistance << std::endl;
        ofs << queryOutput.maxCurvature << std::endl;
        ofs << queryOutput.distancesFirst.size() << " " << ::print(queryOutput.distancesFirst) << std::endl;
        ofs << queryOutput.curvaturesFirst.size() << " " << ::print(queryOutput.curvaturesFirst) << std::endl;
        ofs << queryOutput.distancesSecond.size() << " " << ::print(queryOutput.distancesSecond) << std::endl;
        ofs << queryOutput.curvaturesSecond.size() << " " << ::print(queryOutput.curvaturesSecond) << std::endl;
      }
      LOG_LINE << "saved to " << filename;
    }

    /**
    */  
    void PathPlanningComparisonQuery::loadResults(const char* filename)
    {
      std::ifstream ifs (filename);
      std::string line;
      getlineThrows(ifs, line);
      std::string pathFirst;
      getlineThrows(ifs, pathFirst);
      std::string pathSecond;
      getlineThrows(ifs, pathSecond);
      getlineThrows(ifs, line);
      const size_t N = atoi(line.c_str());
      mOutput.resize(N);
      for (size_t i(0); i<N; ++i)
      {
        mOutput[i].pathNameFirst = pathFirst;
        mOutput[i].pathNameSecond = pathSecond;
        getlineThrows(ifs, mOutput[i].sceneName);
        getlineThrows(ifs, line);
        mOutput[i].maxDistance = atof(line.c_str());
        getlineThrows(ifs, line);
        mOutput[i].maxCurvature = atof(line.c_str());
        // first
        getlineThrows(ifs, line);
        {
          std::stringstream ss(line);
          size_t M;
          ss >> M;
          mOutput[i].distancesFirst.resize(M);
          for (size_t j(0); j<M; ++j)
          {
            ss >> mOutput[i].distancesFirst[j];
          }
        }
        getlineThrows(ifs, line);
        {
          std::stringstream ss(line);
          size_t M;
          ss >> M;
          mOutput[i].curvaturesFirst.resize(M);
          for (size_t j(0); j<M; ++j)
          {
            ss >> mOutput[i].curvaturesFirst[j];
          }
        }
        // second
        getlineThrows(ifs, line);
        {
          std::stringstream ss(line);
          size_t M;
          ss >> M;
          mOutput[i].distancesSecond.resize(M);
          for (size_t j(0); j<M; ++j)
          {
            ss >> mOutput[i].distancesSecond[j];
          }
        }
        getlineThrows(ifs, line);
        {
          std::stringstream ss(line);
          size_t M;
          ss >> M;
          mOutput[i].curvaturesSecond.resize(M);
          for (size_t j(0); j<M; ++j)
          {
            ss >> mOutput[i].curvaturesSecond[j];
          }
        }
      }
      LOG << "loaded " << filename << "\n";
      LOG << "   Scenes: " << mOutput.size() << "\n";
      for (size_t i(0); i<mOutput.size(); ++i)
      {
        LOG << "   name: " << mOutput[i].sceneName << "\n";
        LOG << "   numel distances first   " << mOutput[i].distancesFirst.size() << "\n";
        LOG << "   numel curvatures first  " << mOutput[i].curvaturesFirst.size() << "\n";
        LOG << "   numel distances second  " << mOutput[i].distancesSecond.size() << "\n";
        LOG << "   numel curvatures second " << mOutput[i].curvaturesSecond.size() << "\n";
      }
      LOG_LINE << "";
    }

    void PathPlanningComparisonQuery::visualize(EvaluationWindow* pWindow)
    {
      // create 4 plots
      // first column
      const size_t N = mOutput.size();
      for (size_t i(0); i<N; ++i)
      {
        auto widget = std::make_shared<QWidget>();
        auto layout = new QGridLayout();
        widget->setLayout(layout);
        fs::path p(mOutput[i].sceneName.c_str());
        auto sceneStr = (boost::format("%s: distances") % p.parent_path().stem().string().c_str()).str();
        // first columnd paths plot
        auto pPlot = std::make_unique<MultipleDoubleVectorPlot>();
        {
          pPlot->setTitle(sceneStr.c_str());
          pPlot->setXLabel("path length (mm)");
          pPlot->setYLabel("distance to risk structures (mm)");
          pPlot->setNumberOfGraphs(3);
          pPlot->setData(0, mOutput[i].distancesFirst);
          pPlot->setData(1, mOutput[i].distancesSecond);
          const size_t size = std::max(mOutput[i].distancesFirst.size(), mOutput[i].distancesSecond.size());
          std::vector<double> safetyDist(size, mOutput[i].maxDistance);
          pPlot->setData(2, safetyDist);
          pPlot->setGraphTitle(0, mOutput[i].pathNameFirst.c_str());
          pPlot->setGraphTitle(1, mOutput[i].pathNameSecond.c_str());
          pPlot->setGraphTitle(2, "Safety Distance");
          pPlot->adjust();
          layout->addWidget(pPlot.release(), 0, 0, 1, 1);
        }
        // second column boxplot
        auto pBoxPlot = std::make_unique<BoxPlots>();
        {
          pBoxPlot->setTitle(sceneStr.c_str());
          pBoxPlot->setYLabel("distance to risk structures (mm)");
          const size_t N = mOutput.size();
          pBoxPlot->addBox(mOutput[i].distancesFirst);
          pBoxPlot->addBox(mOutput[i].distancesSecond);          
          pBoxPlot->setSampleName(0, mOutput[i].pathNameFirst.c_str());
          pBoxPlot->setSampleName(1, mOutput[i].pathNameSecond.c_str());
          pBoxPlot->adjust();
          layout->addWidget(pBoxPlot.release(), 0, 1, 1, 1);
        }
        pWindow->addPlot(widget);
      }
      // results together
      auto pBoxPlot = std::make_unique<BoxPlots>();
      {
        auto widget = std::make_shared<QWidget>();
        auto layout = new QGridLayout();
        widget->setLayout(layout);
        if (mOutput.empty())
          pBoxPlot->setTitle("overview");
        else
          pBoxPlot->setTitle(mOutput.back().pathNameFirst.c_str());
        pBoxPlot->setYLabel("distance to risk structures (mm)");
        for (size_t i(0); i<N; ++i)
        {          
          std::vector<double> space;
          pBoxPlot->addBox(mOutput[i].distancesFirst);
          pBoxPlot->addBox(mOutput[i].distancesSecond);
          pBoxPlot->addBox(space);
          pBoxPlot->setSampleName(3*i, "N");
          pBoxPlot->setSampleName(3*i+1, "L");
          pBoxPlot->setSampleName(3*i+2, "");
          pBoxPlot->adjust();          
        }
        auto* qplot = pBoxPlot->getPlot();        
        // create data axis x,y,
        QVector<double> xTime(3*N+1, 0); // +1 looks nicer
        QVector<double> yVal(3*N+1);
        const double val = mOutput.empty() ? 0 : mOutput.back().maxDistance;
        for (size_t k(0); k<3*N+1; ++k)
        {
          xTime[k] = k;
          yVal[k]  = val;
        }
        //
        QCPGraph *graph = qplot->addGraph(qplot->axisRect()->axis(QCPAxis::atBottom), qplot->axisRect()->axis(QCPAxis::atLeft));      
        graph->setPen(QPen(Qt::red));
        graph->setData(xTime, yVal);
        graph->rescaleAxes();        
        qplot->axisRect()->axis(QCPAxis::atLeft, 0)->setRangeLower(0);
        layout->addWidget(pBoxPlot.release(), 0, 0, 1, 1);
        pWindow->addPlot(widget);
      }            
    }
  }
}