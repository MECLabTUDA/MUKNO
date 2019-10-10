#pragma once
#include "BaseController.h"

#include "MukAppModels/AlgorithmModel.h"

#include "MukQt\AlgConnection.h"

class QListWidgetItem;

namespace gris
{
  namespace muk
  {
    class TabImaging;
    class AlgGraphicsView;
    class AlgItemHandler;
    class AlgorithmManager;

    /**
    */
    class AlgorithmController : public BaseController
    {
      Q_OBJECT

      public:
        AlgorithmController();
        ~AlgorithmController() = default;

      public:
        virtual void initialize();
        virtual void setupConnections();

      public:
        void initTabImaging();

      public :
        // TabImaging/AlgGraphicsView
        void newAlgorithm();
        void addAlgorithm(const std::string& name, const QPointF& position);
        void deleteAlgorithm(unsigned int id);
        void addConnection(AlgPort *p1, AlgPort *p2);
        void deleteConnection(AlgPort *p1, AlgPort *p2);
        void showAlgProperties(unsigned int id);
        void showOutput(unsigned int algoId, bool isOutput, unsigned int portId);
        void executePipeline();
        void saveAlgorithm(const std::string& filename) const;
        void loadAlgorithm(const std::string& filename, bool badPropertiesThrow = true);
        void createAlgorithmFromDrop(const std::string& str, const QPointF& pos);
        void setAlgorithmPropFromDrop(const std::string& str, unsigned int id);

      private:
        TabImaging* mpTabImaging;
    };
  } // namespace muk
} // namespace gris