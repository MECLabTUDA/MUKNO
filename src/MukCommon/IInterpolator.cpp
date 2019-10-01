#include "private/muk.pch"
#include "IInterpolator.h"

#include "MukCommon\MukException.h"

#include <algorithm>
#include <functional>
#include <iostream>

namespace
{
  using namespace gris::muk;
}

namespace gris
{
namespace muk
{
  /**
  */
  IInterpolator::IInterpolator()
    : mInterpolated(false)
    , mKappa(0)
    , mPointsPerSegment (5)
    , mResolution(0.1)
    , mInterpolationType(resolution)
  {
    appendProperties();
  }
  
  std::vector<std::string> IInterpolator::getInterpolationTypes()
  {
    std::vector<std::string> ret;
    for (int i(0); i<IInterpolator::N_Types; ++i)
    {
      ret.push_back(InterpolationTypesAsStrings[i]);
    }
    return ret;
  }

  /**
  */
  IInterpolator::IInterpolator(IInterpolator&& o)
  {
    mInterpolated = o.mInterpolated;
    mKappa = o.mKappa;
    mPointsPerSegment = o.mPointsPerSegment;
    mResolution = o.mResolution;
    mInterpolationType = o.mInterpolationType;
    mInput = std::move(o.mInput);

    appendProperties();
  }

  /**
  */
  IInterpolator& IInterpolator::operator=(IInterpolator&& o)
  {
    if (this!=&o)
    {
      mInterpolated = o.mInterpolated;
      mKappa        = o.mKappa;
      mPointsPerSegment = o.mPointsPerSegment;
      mResolution   = o.mResolution;
      mInterpolationType = o.mInterpolationType;
      mInput        = std::move(o.mInput);
    }
    return *this;
  }   

  /**
  */
  void IInterpolator::appendProperties()
  {
    declareProperty<double>("Curvature",
      std::bind(&IInterpolator::setKappa, this, std::placeholders::_1), 
      std::bind(&IInterpolator::getKappa, this));
    declareProperty<size_t>("PointsPerSegment",
      std::bind(&IInterpolator::setPointsPerSegment, this, std::placeholders::_1), 
      std::bind(&IInterpolator::getPointsPerSegment, this));
    declareProperty<double>("Resolution",
      std::bind(&IInterpolator::setResolution, this, std::placeholders::_1), 
      std::bind(&IInterpolator::getResolution, this));
    declareProperty<EnInterpolationTypes>("InterpolationType",
      std::bind(&IInterpolator::setInterpolationType, this, std::placeholders::_1),
      std::bind(&IInterpolator::getInterpolationType, this));
  }

  /**
  */
  void IInterpolator::clone(const IInterpolator* pOther)
  {
    mKappa = pOther->getKappa();
    mInput = pOther->getInput();
    mPointsPerSegment   = pOther->getPointsPerSegment();
    mResolution         = pOther->getResolution();
    mInterpolationType  = pOther->getInterpolationType();
    mInterpolated = false;
  }

  const char* IInterpolator::InterpolationTypesAsStrings[N_Types]
  {
    "controlPoints",
    "samples",
    "pointsPerSegment",
    "resolution"
  };

  int IInterpolator::typeFromString(const char* str)
  {
    auto strEqual = [&] (const char* lhs, const char* rhs) { return 0 == std::strcmp(lhs, rhs); };    
    const char** iter = std::find_if(InterpolationTypesAsStrings, InterpolationTypesAsStrings+N_Types, std::bind(strEqual, str, std::placeholders::_1) );
    if (iter == InterpolationTypesAsStrings+N_Types)
      return -1;
    else
      return static_cast<int>( std::distance(InterpolationTypesAsStrings, iter) );
  }

  /**
  */
  void IInterpolator::setInput(const MukPath& p) 
  {
    mInput = p; 
    mInterpolated = false; 
  }

  /**
  */
  IInterpolator::result_type IInterpolator::getInterpolation() const 
  {
    if ( ! mInterpolated)
    {
      throw MUK_EXCEPTION_SIMPLE("The Interpolator needs te be updated first!");
    }
    return getInterpolation(mInterpolationType); 
  }

  /**
  */
  IInterpolator::InterpolatedPoses IInterpolator::getInterpolatedPoses() const
  {
    if ( ! mInterpolated)
    {
      throw MUK_EXCEPTION_SIMPLE("The Interpolator needs te be updated first!");
    }
    return getInterpolatedPoses(mInterpolationType); 
  }

  /**
  */
  IInterpolator::InterpolatedPoses IInterpolator::getInterpolatedPoses(EnInterpolationTypes) const
  {
    return InterpolatedPoses(); 
  }
}
}

namespace std
{
  std::ostream& operator<< (std::ostream& os, const gris::muk::IInterpolator::EnInterpolationTypes& obj)
  {
    const int i (obj);    
    return os << IInterpolator::InterpolationTypesAsStrings[i];
  }

  /**
  */
  std::istream& operator>> (std::istream& is, gris::muk::IInterpolator::EnInterpolationTypes& obj)
  {
    std::string tmp;
    is >> tmp;
    auto begin = IInterpolator::InterpolationTypesAsStrings;
    auto end   = IInterpolator::InterpolationTypesAsStrings + IInterpolator::EnInterpolationTypes::N_Types;
    auto iter  = std::find_if(begin, end, [&] (const char* str) { return tmp == str; });
    if (iter == end)
    {
      LOG_LINE << "WARNING: could not interpret '" << tmp << "' as IInterpolator::EnInterpolationTypes";
    }
    else
    {
      obj = IInterpolator::EnInterpolationTypes(std::distance(begin, iter));
    }
    return is;
  }
}