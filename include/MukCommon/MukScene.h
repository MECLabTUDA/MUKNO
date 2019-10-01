#pragma once

#include "muk_common_api.h"

#include "Bounds.h"
#include "version.h"
#include "IInterpolator.h"
#include "INavigationContainer.h"

#include "gstd/logger.h"

#include <string>
#include <memory>

namespace gris
{
  class XmlNode;

namespace muk
{
  class AbstractObject;
  class SystemCalibration;
  class PathCollection;
  class MukPath;
  class MukObstacle;
  class ICollisionDetector;
  class IPathPlanner;
  class IPathOptimizer;
  class IInterpolator;
  class INavigator;  

  class MUK_COMMON_API MukScene
  {
    public:
      MukScene();
      ~MukScene();

    public:
      friend class MukSceneLegacyIO;
      void save(const std::string& filename) const;      
      void load(const std::string& filename);
      void swap(MukScene& scene);
      void reset();

    public:
      void                  setName(const std::string& name);
      const std::string&    getName()                         const { return mSceneName; }
      void                  setLocalBasePath(const std::string& basePath)                 { mLocalBasePath = basePath; }
      const std::string&    getLocalBasePath()                                    const   { return mLocalBasePath; }
      const IPathPlanner*   getPlanner()                                          const   { return mpPlanner.get(); }
      IPathPlanner*         getPlanner()                                                  { return mpPlanner.get(); }
      void                  setPlanner(std::unique_ptr<IPathPlanner> pObj);
      const IPathOptimizer*    getPruner()                                           const   { return mpPruner.get(); }
      IPathOptimizer*          getPruner()                                                   { return mpPruner.get(); }
      void                  setPruner(std::unique_ptr<IPathOptimizer> pObj);
      const IInterpolator*  getInterpolator()                                     const   { return mpInterpolator.get(); }
      IInterpolator*        getInterpolator()                                             { return mpInterpolator.get(); }
      void                  setInterpolator(std::unique_ptr<IInterpolator> pObj)          { mpInterpolator = std::move(pObj); }
      
      void  setSystemCalibration(std::shared_ptr<SystemCalibration> pObj)   { mpSystemCalibration = pObj; }
      void  setCollisionDetector(std::shared_ptr<ICollisionDetector> pObj);
      const SystemCalibration&            getSystemCalibration() const   { return *mpSystemCalibration.get(); }      
      std::shared_ptr<ICollisionDetector> getCollisionDetector() const   { return mpCollisionDetector; }     

      void                               setNavigation(INavigationContainer * const c)        { mpNavigation = c; }
      INavigationContainer * const       getNavigation()                                      { return mpNavigation; }
      const INavigationContainer * const getNavigation()                                const { return mpNavigation; }


    public:
      bool                         hasPathKey(const std::string& key) const;      
      PathCollection&              getPathCollection(const std::string& key);
      std::vector<std::shared_ptr<PathCollection>>& getPathCollections() { return mPaths; }

      void            insertPathCollection(const std::string& key);
      void            deletePathCollection(const std::string& key);

      std::shared_ptr<MukObstacle> getObstacle(const std::string& key) const;
      void               insertObstacle(std::shared_ptr<MukObstacle> obs);
      void               deleteObstacle(const std::string& key);
      void               setObstacleActive(const std::string& key, bool on);

    public:
      std::vector<std::string> getPathKeys()     const;
      std::vector<std::string> getObstacleKeys() const;
            
    private:
      std::string mSceneName;
      std::string mLocalBasePath;

      std::shared_ptr<SystemCalibration>  mpSystemCalibration;
      std::shared_ptr<ICollisionDetector> mpCollisionDetector;
      std::unique_ptr<IPathPlanner>    mpPlanner;
      std::unique_ptr<IPathOptimizer>     mpPruner;
      std::unique_ptr<IInterpolator>   mpInterpolator;

      // fix, for now just save the name!
      //std::shared_ptr<INavigator>      mpNavigator;
      INavigationContainer*             mpNavigation;
      // Data
      std::vector<std::shared_ptr<PathCollection>> mPaths;
      std::vector<std::shared_ptr<MukObstacle>>    mObstacles;
      std::vector<std::shared_ptr<AbstractObject>> mAbstractObjects;

      std::string mActivePathCollection;
  };

}
}