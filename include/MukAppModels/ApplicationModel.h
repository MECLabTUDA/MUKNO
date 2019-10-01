#pragma once

#include "BaseModel.h"

#include "MukCommon/Bounds.h"
#include "MukCommon/MukPath.h"
#include "MukCommon/Waypoints.h"
#include "MukCommon/MukScene.h"
#include "MukCommon/MukException.h"
#include "MukCommon/ICollisionDetector.h"
#include "MukCommon/IPathPlanner.h"
#include "MukCommon/IPathOptimizer.h"
#include "MukCommon/IInterpolator.h"
#include "MukCommon/INavigationContainer.h"
#include "MukCommon/SystemCalibration.h"

#include "MukImaging/MukImage.h"

#include "gstd/logger.h"

#include <itkImage.h>
#include <itkCastImageFilter.h>
#include <itkSmoothingRecursiveGaussianImageFilter.h>

#include <map>
#include <memory>

namespace gris
{
  class XmlNode;

  namespace muk
  {
    class MukQToolBar;
    class MukQMenuBar;
    class TabPlanning;

    /** \brief Workflow of the general application

      determines behavior of e.g. load / save commands
    */
    class MUK_APP_API ApplicationModel : public BaseModel
    {
      public:
        ApplicationModel();

      public:
        virtual const char* name() const { return "ApplicationModel"; }

      public:
        void handleException(const MukException& e);
        void handleException(const std::exception& e);

      public:
        std::shared_ptr<MukScene> getScene() { return mpScene; }

      public:
        void reset();
        void saveScene(const std::string& directory) const;
        void loadScene(const std::string& filename);
        void savePath(const std::string& filename);
        void loadPath(const std::string& filename);
      
        void addObstacle (std::shared_ptr<MukObstacle> pObj);
        void loadObstacle(const std::string& filename);
        void saveObstacle(const std::string& filename, const std::string& name);
        void deleteObstacle(const std::string& name);

      public:
        void setCtData(ImageInt3D::Pointer pData) { mpCtData = pData; }

        void setExecutableDir(const std::string& str);
        const std::string& getExecutableDir() const;
        void setWorkingDir(const std::string& str);
        const std::string& getWorkingDir() const;

      private:
        // Data
        std::shared_ptr<MukScene> mpScene;

        ImageInt3D::Pointer mpCtData;
	      std::shared_ptr<std::map<int, ImageInt3D::Pointer>> mpSegmentationData;

        std::string mExecutableDir;
        std::string mWorkingDir;
    };
  }
}
