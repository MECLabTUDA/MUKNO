#pragma once
#include <string>
#include <vector>

namespace gris
{
  namespace opt
  {
    /** \brief A Wrapper for variables created by Gurobi
    */
    class VariableRepresentation
    {
      friend class Variable;

    public:
      VariableRepresentation(int index, const std::string& name, void* creator)
        : mIndex(index)
        , mName(name)
        , mRemoved(false)
        , mpCreator(creator)
      {
      }

    public:
      void                setIndex(int i)    { mIndex = i; }
      void                setRemoved(bool b) const { mRemoved = b; } // TODO: fix broken trajopt API

      const std::string&  getName()  const { return mName; }
      int                 getIndex() const { return mIndex; }
      bool                getRemoved()  const { return mRemoved; }

    private:
      std::string mName;
      int         mIndex;
      mutable bool mRemoved;
      void*       mpCreator; // pointer to gurobi interface
    };

    /** \brief A Variable to be optimized by a ConvexOptimizationSolver
      
      The actual variable is hidden inside Gurobi. 
      This wrapper implementation just saves the necessary information (name, index of the value within a vector of doubles, existence flag) to access it.
    */
    class Variable
    {
      public:
        Variable() : mpRep(nullptr) {}
        Variable(VariableRepresentation* var_rep) : mpRep(var_rep) {}
        Variable(const Variable& other) : mpRep(other.mpRep)  {}
        
      public:
        double value(const double* x)              const { return x[mpRep->mIndex]; }
        double value(const std::vector<double>& x) const { return x[mpRep->mIndex]; }

        const VariableRepresentation* getRep() const  { return mpRep; }
              VariableRepresentation* getRep()        { return mpRep; }

      public:
        std::ostream& operator<<(std::ostream& os) const;
        friend std::ostream& operator<<(std::ostream& os, const Variable& v) { return v.operator<<(os); }

      private:
        VariableRepresentation* mpRep;
    };
  }
}