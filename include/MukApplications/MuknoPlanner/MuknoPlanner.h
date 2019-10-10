#pragma once

#include "MukAppModels/AppModels.h"
#include "AppControllers.h"

#include "MukQt/MuknoPlannerMainWindow.h"

#include <QMainWindow>

#include <memory>
#include <mutex>

namespace gris
{
  namespace muk
  {
    // gui classes
    class HandlerTextEditLog;
    class QtLogWrapper;   

    /**
    */
    class MuknoPlanner : public QMainWindow
    {
      Q_OBJECT

      public:
        static std::unique_ptr<MuknoPlanner> create(const std::string& exe);

      public:
        ~MuknoPlanner();
        
      private:
        MuknoPlanner();
        MuknoPlanner(const MuknoPlanner& o) = delete;
        MuknoPlanner& operator=(const MuknoPlanner& o) = delete;

      public:
        virtual void closeEvent (QCloseEvent *event);

      public:
        void showInStartUpMode();
        void setupConnections();
        void handleFileInput(const std::string& filename);

      private:
        void init(const std::string& exePath);
        void writePropertyFile() const;

      private:
        // base structure
        std::unique_ptr<AppModels> mpModels;
        std::unique_ptr<AppControllers> mpControls;
        // owns gui
        std::unique_ptr<MuknoPlannerMainWindow>   mpMasterWindow;
        
        // Handlers of the different layouts/Widgets        
        std::unique_ptr<HandlerTextEditLog>       mpHandlerLog;
        std::unique_ptr<QtLogWrapper>             mpWrapperLog;
    };

  }
}
