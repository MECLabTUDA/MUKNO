#include "private/muk.pch"
#include "private/types.h"
#include "private/ValidityCheckerKdTreeNearestNeighbor.h"

#include "MukCommon/MukException.h"
#include "MukCommon/CollisionDetectorKdTree.h"
#include "MukCommon/IBounds.h"
#include "MukCommon/Bounds.h"

namespace ob = ompl::base;

namespace gris
{
	namespace muk
	{
		// ============== PIMPL ===============
		struct ValidityCheckerKdTreeNearestNeighbor::Impl
		{
			Impl();
			~Impl();

			double mDistance;
      double mClearance;
		};

		ValidityCheckerKdTreeNearestNeighbor::Impl::Impl()
			: mDistance(0.0)
		{
		}

		ValidityCheckerKdTreeNearestNeighbor::Impl::~Impl()
		{
		}

		// ============== CLASS IMPL ===============
		ValidityCheckerKdTreeNearestNeighbor::ValidityCheckerKdTreeNearestNeighbor(ompl::base::SpaceInformation* si)
			: IStateValidityChecker(si)
			, mp(std::make_unique<Impl>())
		{
		}


		ValidityCheckerKdTreeNearestNeighbor::ValidityCheckerKdTreeNearestNeighbor(const ompl::base::SpaceInformationPtr& si)
			: IStateValidityChecker(si)
			, mp(std::make_unique<Impl>())
		{
		}

		ValidityCheckerKdTreeNearestNeighbor::~ValidityCheckerKdTreeNearestNeighbor()
		{
		}

		/**
		*/
		void ValidityCheckerKdTreeNearestNeighbor::setMaxDistance(double distance)
		{
			mp->mDistance = distance;
		}

		/**
		*/
		double ValidityCheckerKdTreeNearestNeighbor::getMaxDistance() const
		{
			return mp->mDistance;
		}

		/**
		*/
		double ValidityCheckerKdTreeNearestNeighbor::clearance(const ompl::base::State* pState) const
		{
      return mp->mClearance;
			/*const auto* state = pState->as<og::MukStateType>();
			auto p = Vec3d(state->getX(), state->getY(), state->getZ());
			if (mpCollisionDetector->hasNeighbors(p, mp->mDistance))
				return false;
			return true;*/
		}

		/**
		*/
		bool ValidityCheckerKdTreeNearestNeighbor::isValid(const ompl::base::State* pState) const
		{
			const auto* state = pState->as<og::MukStateType>();
			auto p = Vec3d(state->getX(), state->getY(), state->getZ());
			if (!mpBounds->isInside(p))
				return false;

      Vec3d nn;
      bool hasNeighbor = mpCollisionDetector->nearestNeighbor(p, nn);
      if (hasNeighbor)
      {
        mp->mClearance = (nn - p).norm();
        if (mp->mClearance < mp->mDistance)
          return false;
      }
			return true;
		}

	}
}