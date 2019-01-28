#pragma once
#include "BaseModel.h"

#include "MukCommon/MukTransform.h"
#include "MukCommon/INavigator.h"
#include "MukCommon/MukState.h"
#include "MukCommon/MukPath.h"

#include "MukVisualization/VisCoordinateSystemCollection.h"

#include "MukQt/VtkWindow.h"

#include "gstd/DynamicProperty.h"

#include <vtkImageData.h>

#include <memory>

namespace gris
{
  namespace muk
  {
    struct CameraConfiguration;
    class MedicalMultiViewWidget;
    class MukScene;
    class VisScene;
    class VisAbstractObject;
    class VisStateRegion;
    class MukQMenuBar;
    class MukQToolBar;
    class MuknoPlannerMainWindow;
    class NavigationThread;

    /** \brief Appends Visualization to Objects like MukPath
    */
    class MUK_APP_API VisualizationModel : public BaseModel
    {
      public:
        VisualizationModel();
        virtual ~VisualizationModel();

      public:
        virtual const char* name() const { return "VisualizationModel"; }

      public:
        void setScene(std::shared_ptr<MukScene> pScene);
        void setVisWindow(MedicalMultiViewWidget* pVisWindow);
        MedicalMultiViewWidget* getVisWindow() { return mpVisWindow; }
        VisScene* getVisScene()  const { return mpVisScene.get(); }
        
      public:
        void reset();
        //
        void addPath   (const std::string& name, size_t idx=0);
        void updatePath(const std::string& name, size_t idx=0);
        void deletePath(const std::string& name, size_t idx=0);
        void clearPaths(const std::string& name);
        void addPathCollection(const std::string& name);
        void deletePathCollection(const std::string& name);
        //
        void addObstacle(const std::string& name);
        void deleteObstacle(const std::string& name);
        //
        std::vector<std::string> getAbstractObjectKeys() const;
        std::shared_ptr<VisAbstractObject> getAbstractObject(std::string key) const;
        bool hasAbstractObject(const std::string& name)  const;
        void addAbstractObject(std::shared_ptr<VisAbstractObject> pObj);
        void deleteAbstractObject(const std::string& name);
        void clearAbstractObjects();
        //
        void setRobot(const std::string& name);
        void setRobotState(const std::string& pathCollection, size_t pathIdx, size_t stateIdx);
        // slots for Visualization updates
		    void deleteTabObjects(int index);

        // visualization of navigation procedure
      public:
        void setRobotTransform(const MukTransform& transform);
        void setAdaptedPath(const MukPath& path);
        void setCurrentTargetOrientation(const Vec3d& state);
        // update navigation visualization
        void updateTransform(const MukTransform& transform);
        void updateNavigatorFeature(const INavigator::NavigatorFeature& feature, const bool& state);
        // slots for synchronization of NavigatorState with Visualization
        void updateNavState_Initialization(const bool pState);
        // update requests
        void updateRobot();
        void updateTrajectory();
        void updateTargetOrientation();

      public:
        void showBounds();
        void setDefaultFocus();
        void focus();
        void focusOnWaypoint();
        void render();
        //
        friend class VisualizationModelLegacyIO;
        void save(const std::string& filename) const;
        void load(const std::string& filename);
        void synchronize(const MukScene& scene);

     private:
        void setCameraConfiguration(const CameraConfiguration& config);
        const CameraConfiguration& getCameraConfiguration() const;        
                
      private:
        MedicalMultiViewWidget*       mpVisWindow;
        std::shared_ptr<MukScene>  mpScene;
        std::unique_ptr<VisScene>  mpVisScene;

        std::shared_ptr<CameraConfiguration> mDefaultView;
        bool mDefaultViewSet;

        // variables to hold Visualization info
        VisCoordinateSystemCollection mCoordinateSystems;
        vtkSmartPointer<vtkTransform> mpRobotTransform;
        MukPath  mAdaptedPath;
        Vec3d    mCurrentTargetOrientation;
    };
  }
}
