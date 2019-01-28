#pragma once

#include "muk_visualization_api.h"

#include "VisualObject.h"
#include "VisTimeDelays.h"
#include "VisCoordinateSystem.h"

#include <mutex>

namespace gris
{
namespace muk
{

  class MukTransform;

  class MUK_VIS_API VisCoordinateSystemCollection : public VisualObject
  {
  protected:
    typedef std::pair<MukTransform, std::unique_ptr<VisCoordinateSystem>> CoordinateSystemPair;
    typedef std::vector<CoordinateSystemPair> List;
    typedef List::iterator Iterator;
    typedef std::chrono::steady_clock clock;

    typedef std::recursive_mutex        MutexType;
    typedef std::lock_guard<MutexType>  LockType;

  public:
    explicit VisCoordinateSystemCollection();
    explicit VisCoordinateSystemCollection(const VisTimeDelays& timedelays);
    ~VisCoordinateSystemCollection();

  public:
    virtual void        update();

    void                updateBeforeRender();
    virtual void        setRenderer(vtkRenderer* pRenderer);

  public:
    void updateCoordinateSystem(const MukTransform& transform);
    void deleteCoordinateSystem(const std::string& name);
    void clearCoordinateSystems();
  private:
    bool findCoordinateSystem(const std::string& name);

  private:
    List        mTransforms;
    Iterator    mLastSearchIterator;
    std::string mLastSearchName;
    bool        mLastSearchValid;

    VisTimeDelays                    mTimeDelays;

    MutexType  mTransformsMutex;
  };
}
}