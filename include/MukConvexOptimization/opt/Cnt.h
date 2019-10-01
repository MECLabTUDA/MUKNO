#pragma once

#include <string>

namespace gris
{
  namespace opt
  {
    /**
    */
    enum ConstraintType 
    {
      enEQ,
      enINEQ,
      enUnknownConstraint
    };

    /** \brief Internal representation of a constraint for a convex optimization problem
    */
    class ConstraintRepresentation 
    {
      public:
        ConstraintRepresentation(int index, void* creator)
          : mIndex(index)
          , mRemoved(false)
          , mpCreator(creator)
          , mType(enUnknownConstraint)
        {
        }

      public:
        void            setIndex(int i) { mIndex = i; }  
        void            setRemoved(bool b) const { mRemoved = b; } // TODO: fix broken trajopt API

        int             getIndex()    const { return mIndex; }
        ConstraintType  getType()     const { return mType; }
        bool            getRemoved()  const { return mRemoved; }
        const std::string& getExpression() const { return mExpr; }

      private:
        int   mIndex;
        mutable bool  mRemoved;
        void* mpCreator;
        ConstraintType mType;
        std::string    mExpr; // todo placeholder
    };

    /**
    */
    class Cnt
    {
      public:
        Cnt()                 : mpRep(nullptr)  {}
        Cnt(ConstraintRepresentation* cnt_rep)  : mpRep(cnt_rep)  {}

      public:
        const ConstraintRepresentation* getRep() const  { return mpRep; }
              ConstraintRepresentation* getRep()        { return mpRep; }

      private:
        ConstraintRepresentation* mpRep;
    };
  }
}
