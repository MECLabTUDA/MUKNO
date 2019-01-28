#include "private/muk.pch"
#include "EvaluationModel.h"
#include "ApplicationModel.h"
#include "PlanningModel.h"
#include "MukAppModels/VisualizationModel.h"

#include "MukCommon/EvaluationFactory.h"
#include "MukCommon/PathCollection.h"
#include "MukCommon/geometry.h"
#include "MukCommon/MukObstacle.h"
#include "MukCommon/ICollisionDetector.h"

#include "MukQt/MukQToolbar.h"
#include "MukQt/TabEvaluation.h"
#include "MukQt/EvaluationWindow.h"

#include "MukVisualization/muk_colors.h"
#include "MukVisualization/VisPathCollection.h"
#include "MukVisualization/VisScene.h"

#include "MukEvaluation/EvaluationDummy.h"
#include "MukEvaluation/statistics.h"

#include <gstd/XmlDocument.h>

#include <vtkSphereSource.h>

namespace
{
  using gris::Vec3d;
  using namespace gris::muk;

  class CollisionDetectionHandler
  {
    public:
      CollisionDetectionHandler();
      ~CollisionDetectionHandler();

    public:
      void initialize(ICollisionDetector* pObj);
      void setupForObstacle(const std::string& key);
      std::pair<size_t, double> distance(const MukPath& path, double pathRadius);
      void finish();
      
    private:
      std::vector<std::pair<std::string, bool>> mPreSetup;
      ICollisionDetector* pDetector;
  };
}

namespace gris
{
namespace muk
{
  /**
  */
  EvaluationModel::EvaluationModel()
    : BaseModel()
    , mEval(std::make_unique<EvaluationDummy>())
  {
  }

  /**
  */
  EvaluationModel::~EvaluationModel()
  {
  }

  /**
  */
  void EvaluationModel::setupConnections(TabEvaluation* tab)
  {
    connect(tab, &TabEvaluation::evaluate, this, &EvaluationModel::evaluate);
    connect(tab, &TabEvaluation::loadXml,  this, &EvaluationModel::setQuery);
    connect(tab, &TabEvaluation::saveXml,  this, &EvaluationModel::saveQuery);
    connect(tab, &TabEvaluation::setEvaluation , this, &EvaluationModel::setEvaluation);

    connect(tab, &TabEvaluation::loadResults,  this, &EvaluationModel::loadResults);
    connect(tab, &TabEvaluation::saveResults,  this, &EvaluationModel::saveResults);
  }

  /**
  */
  void EvaluationModel::evaluate()
  {
    if (mEval.get() != nullptr)
    {
      mpEvalWindow->clearPlots();
      mEval->evaluate();
      mEval->visualize(mpEvalWindow);
    }
  }

  /**
  */
  void EvaluationModel::setQuery(const std::string& text) 
  {
    mEval->setQuery(text);    
  }
  
  /**
  */
  const std::string& EvaluationModel::getQuery() const
  {   
    return mEval->getQuery();
  }

  /**
  */
  void EvaluationModel::saveQuery(const char* filename) const
  {
    const std::string& data = mEval->getQuery();
    auto pDoc = XmlDocument::create("tmp");
    pDoc->fromString(data.c_str());    
    XmlDocument::save(filename, *pDoc);
    LOG_LINE << "saved query file to " << filename;
  }

  /**
  */
  void EvaluationModel::setEvaluation(const char* type) 
  {
    auto newEvaluator = GetEvaluationFactory().create(type);
    newEvaluator->setQuery( mEval->getQuery() );
    mEval.swap(newEvaluator);
  }

  /**
  */
  void EvaluationModel::saveResults(const char* filename) const
  {
    mEval->saveResults(filename);
    LOG_LINE << "saved results to " << filename;
  }

  /**
  */
  void EvaluationModel::loadResults(const char* filename)
  {
    mEval->loadResults(filename);
    mpEvalWindow->clearPlots();
    mEval->visualize(mpEvalWindow);
    LOG_LINE << "results loaded";
  }

}
}
