#include "private/muk.pch"
#include "PathCollection.h"

#include "MukCommon/MukException.h"

#include <boost/format.hpp>

namespace gris
{
  namespace muk
  {
    /**
    */
    PathCollection::PathCollection(const std::string& name)
      : mName(name)
      , mpProbDef(std::make_shared<MukProblemDefinition>())
    {
      initialize();
    }

    /**
    */
    PathCollection::PathCollection(PathCollection&& o)
      : mName(std::move(o.mName))
      , mPaths(o.mPaths)
    {
      initialize();
    }

    /**
    */
    PathCollection& PathCollection::operator=(PathCollection&& o)
    {
      if (this!=&o)
      {
        swap(o);
        appendProperties();
      }
      return *this;
    }

    /**
    */
    void PathCollection::appendProperties()
    {
      declareProperty<std::string>("Name",
        std::bind(&PathCollection::setName, this, std::placeholders::_1), 
        std::bind(&PathCollection::getName, this));
    }

    /**
    */
    void PathCollection::initialize()
    {
      appendProperties();
    }

    /** \brief Removes the obstacle key from the list of inactive obstacles.
    */
    void PathCollection::removeObstacle(const std::string& key)
    {
      mInactiveObstacles.erase(std::remove_if(mInactiveObstacles.begin(), mInactiveObstacles.end(), [&] (const auto& str) { return str == key; } ), mInactiveObstacles.end());
    }

    /** \brief Adds the obstacle key to the list of inactive obstacles.
    */
    void PathCollection::addObstacle(const std::string& key)
    {
      mInactiveObstacles.push_back(key);
      std::unique(mInactiveObstacles.begin(), mInactiveObstacles.end());
    }

    /** \brief Returns the list of obstacle keys that shall not be included in the collision detection.
    */
    const std::vector<std::string> PathCollection::getObstacles() const
    {
      return mInactiveObstacles;
    }

    /**
    */
    void PathCollection::swap(PathCollection& o)
    {
      mName.swap(o.mName);
      std::swap(mPaths, o.mPaths);
      std::swap(mpProbDef, o.mpProbDef);
      std::swap(mInactiveObstacles, o.mInactiveObstacles);
    }

    /**
    */
    void PathCollection::setPath(size_t idx, const MukPath& path)
    {    
      if (idx >= mPaths.size())
      {
        throw MUK_EXCEPTION("Path index does not yet exist", (boost::format("%d") % idx).str().c_str());
      }
      else
      {
        mPaths[idx] = path;
      }
    }

    /**
    */
    void PathCollection::insertPath(const MukPath& path)
    {
      mPaths.push_back(path);
      mPaths.back().setRadius(mpProbDef->getRadius()); // quickfix, do not know yet, how to handle radius for paths.
    }


  }
}