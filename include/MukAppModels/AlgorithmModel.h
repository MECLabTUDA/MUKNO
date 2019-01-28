#pragma once
#include "BaseModel.h"
#include "MukAlgorithms\AlgorithmManager.h"
#include "MukAlgorithms\AlgorithmWrapper.h"
#include "MukAlgorithms\AlgorithmFactory.h"

namespace gris
{
  namespace muk
  {
    /**
    */
    class MUK_APP_API AlgorithmModel : public BaseModel
    {
      public:
        using uint = unsigned int;

      public:
        AlgorithmModel();
        ~AlgorithmModel();

      public:
        virtual const char* name() const { return "AlgorithmModel"; }

      public:
        void                        clear();
        unsigned int				        addAlgorithm(const std::string& name);
        const AlgorithmWrapper&     getAlgorithm(unsigned int id) const;
        AlgorithmWrapper&           getAlgorithm(unsigned int id);
        void                        deleteAlgorithm(unsigned int id);

        void                        addConnection(unsigned int sourceID, unsigned int sourcePort, unsigned int destinationID, unsigned int destinationPort);
        void                        deleteConnection(unsigned int sourceID, unsigned int sourcePort, unsigned int destinationID, unsigned int destinationPort);

        std::vector<std::string>    listItems();
        std::pair<uint, uint>       getSource(const std::pair<uint, uint>& destination);
        void                        update();

      public:
        const AlgorithmManager&     getManager() const { return mAlgorithmManager; }

      public:
        void                        saveAlgorithm(const std::string& filename) const;
        void                        loadAlgorithm(const std::string& filename, bool badPropertiesThrow = true);

      private:
        AlgorithmManager mAlgorithmManager;
        std::map<std::pair<uint, uint>, std::pair<uint, uint>> mConnections;
    };

  } // namespace muk
} // namespace gris
