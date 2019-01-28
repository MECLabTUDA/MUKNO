#pragma once

#include "muk_common_api.h"
#include "MukState.h"

#include "gstd/DynamicProperty.h"

#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/deque.hpp>

#include <deque>

namespace gris
{
  namespace muk
  {
    /* \brief Waypoints of a path planner that have to be visited
      
      Constructor enforces that Start and Goal always exist.
      Deleting those is impossible, but they can be set invalid.
    */
    class /*MUK_COMMON_API*/ Waypoints : public gstd::DynamicProperty
    {      
      public:
        Waypoints();
        Waypoints(const Waypoints& o);
        Waypoints& operator=(const Waypoints& o);
        ~Waypoints();

      public:
        void resize(size_t );
        void setStartState(const MukState& s)   { mStates.front() = s;  }
        void setGoalState (const MukState& s)   { mStates.back() = s;   }
        void setPoint     (const MukState& s, size_t index)  { mStates.at(index) = s; }
        void insertPoint  (const MukState& s, size_t index)  { mStates.insert(mStates.begin()+index, s); }
        void removePoint  (size_t index);

        const MukState& getStartPoint() const   { return mStates.front(); }
        const MukState& getGoalPoint() const    { return mStates.back();  }
        const MukState& getPoint(size_t index) const { return mStates.at(index); }

        MukState& getPoint(size_t index)     { return mStates.at(index); }
        MukState& getStartPoint()            { return mStates.front(); }
        MukState& getGoalPoint()             { return mStates.back();  }

        size_t size() const  { return mStates.size(); }

        const std::deque<MukState>& states() const { return mStates; }
        std::deque<MukState>&       states()       { return mStates; }

      public:
        void swap(Waypoints& o);
        std::ostream& print(std::ostream& os);

      private:
        //friend MUK_COMMON_API std::ostream & operator<<(std::ostream& os, const Waypoints& gp);
        friend class boost::serialization::access;
        template<class Archive>
        void serialize(Archive& ar, const unsigned int)
        {
          ar & mStates;
        }
        
      private:
        std::deque <MukState> mStates;        

      public:
        static MukState InitStartState;
        static MukState InitGoalState;
    };


    void MUK_COMMON_API swap(Waypoints& a, Waypoints& b);

  }
}


