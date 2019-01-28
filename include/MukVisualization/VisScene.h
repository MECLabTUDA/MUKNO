#pragma once

#include "muk_visualization_api.h"
#include <memory>

namespace gris
{
  namespace muk
  {
    class MukScene;
    class VisPathCollection;
    class VisObstacle;
    class VisAbstractObject;
    class IVisRobot;

    /** \brief collects visual elements of the 3D Window
    */
    class MUK_VIS_API VisScene
    {
      public:
        VisScene() {}        
        ~VisScene();

      public:
        bool                               hasPathCollection(const std::string& name) const;
        void                               setPathCollection(std::shared_ptr<VisPathCollection>& obj);
        std::shared_ptr<VisPathCollection> getPathCollection(const std::string& name) const;
        std::shared_ptr<VisObstacle>       getObstacle(const std::string& name) const;
        std::shared_ptr<VisAbstractObject> getObject(const std::string& name) const;

        void addPathCollection(std::shared_ptr<VisPathCollection> pObj);
        void addObstacle      (std::shared_ptr<VisObstacle> pObj);
        void addObject        (std::shared_ptr<VisAbstractObject> pObj);
        void deletePathCollection(const std::string& name);
        void deleteObstacle      (const std::string& name);
        void deleteObject        (const std::string& name);
        // convenience functions
        void clearObstacles()       { mObstacles.clear(); }
        void clearPathCollections() { mPaths.clear(); }
        void clearObjects()         { mObjects.clear(); }
        
        void        setRobot(std::shared_ptr<IVisRobot> pObj);
        IVisRobot&  getRobot();

        bool hasObject(const std::string& name);

        void reset();

        std::vector<std::string> getAbstractObjectKeys() const;
        std::vector<std::string> getPathCollectionKeys() const;
        std::vector<std::string> getObstacleKeys() const;

        void save(const std::string& filename) const;
        void load(const std::string& filename);

      private:
        std::vector<std::shared_ptr<VisPathCollection>> mPaths;
        std::vector<std::shared_ptr<VisObstacle>>       mObstacles;
        std::vector<std::shared_ptr<VisAbstractObject>> mObjects;
        std::shared_ptr<IVisRobot> mpVisRobot;
    };

  }
}
