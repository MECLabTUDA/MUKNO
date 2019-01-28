#pragma once

#include "Bounds.h"
#include "muk_common_api.h"
#include "MukPath.h"
#include "MukStateRegion.h"
#include "Waypoints.h"
#include "MukProblemDefinition.h"

#include "gstd/dynamicProperty.h"

namespace gris
{
  namespace muk
  {
    /** \brief Combination of a Motion Planning Problem Definition and paths that were successfully created
      
        A PathCollection (the name is stupid now) can basically seen as the representation of the planning of an access path.
        Its problem defintion describes parameters, the paths possible solutions from which the surgeon can pick one.
    */
    class MUK_COMMON_API PathCollection : public gstd::DynamicProperty
    {
      public:
        explicit PathCollection(const std::string& name);
        PathCollection(PathCollection&& o);
        PathCollection& operator=(PathCollection&& o);

        ~PathCollection() {}
        
        PathCollection() = delete;
        PathCollection(const PathCollection&) = delete;
        PathCollection& operator=(const PathCollection&) = delete;

      public:
        void               setName(const std::string& s)       { mName = s; }
        const std::string& getName()                     const { return mName; }
        std::shared_ptr<MukProblemDefinition> getProblemDefinition() const { return mpProbDef; }

      public:
        const std::vector<MukPath>& getPaths()              const { return mPaths; } 
              std::vector<MukPath>& getPaths()                    { return mPaths; }
        void              setPath(size_t idx, const MukPath& path);
        void              insertPath(const MukPath& path);

        void              removeObstacle(const std::string& key);
        void              addObstacle(const std::string& key);
        const std::vector<std::string> getObstacles() const;
        const std::vector<std::string>& getInactiveObstacles() const { return mInactiveObstacles; };
        std::vector<std::string>&       getInactiveObstacles()       { return mInactiveObstacles; };

      public:
        void swap(PathCollection& o);

      private:
        void appendProperties();
        void initialize();
        
      private:
        std::string                           mName;
        std::shared_ptr<MukProblemDefinition> mpProbDef;
        std::vector<std::string>              mInactiveObstacles;
        std::vector<MukPath>                  mPaths;
    };

  }
}
