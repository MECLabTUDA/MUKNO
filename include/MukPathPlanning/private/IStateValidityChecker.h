#pragma once

#include "MukCommon/IBounds.h"
#include "MukCommon/Bounds.h"
#include "MukCommon/ICollisionDetector.h"

#pragma warning (push)
#pragma warning( disable: 4267 ) // ompl -> conversion from size_t to unsigned int 
#pragma warning( disable: 4800 ) // forcing value to bool
#pragma warning( disable: 4996 ) // function (localtime) may be unsafe
#include <ompl/base/StateValidityChecker.h>
#pragma warning (pop)

#include <memory>

namespace gris
{
  namespace muk
  {
    class IStateValidityChecker : public ompl::base::StateValidityChecker
    {
      public:
        IStateValidityChecker(ompl::base::SpaceInformation* si);
        IStateValidityChecker(const ompl::base::SpaceInformationPtr& si);
        virtual ~IStateValidityChecker() {}

      public:      
        void setCollisionDetector(std::shared_ptr<ICollisionDetector> pObj) { mpCollisionDetector = pObj; }
        std::shared_ptr<ICollisionDetector> getCollisionDetector()  const   { return mpCollisionDetector; }

        virtual bool isValid(const ompl::base::State* state) const = 0;

      public:
        virtual void    setMaxDistance(double)                            {}
        virtual double  getMaxDistance() const                            { return std::numeric_limits<double>::max(); }

        void           setBounds(const IBounds* pBounds)   { mpBounds = pBounds; }
        const IBounds& getBounds() const                   { return *mpBounds; }
        
      protected:
        std::shared_ptr<ICollisionDetector> mpCollisionDetector;
        const gris::muk::IBounds* mpBounds;
    };
  }
}