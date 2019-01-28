#pragma once

#include "VisAbstractObject.h"

#include "MukCommon/MukPathGraph.h"

namespace gris
{
  namespace muk
  {

    class MUK_VIS_API VisMukPathGraph : public VisAbstractObject
    {
      public:
        explicit VisMukPathGraph(const std::string& key);
        virtual ~VisMukPathGraph();

        void   setPointSize(double d);
        double getPointSize() const { return mPointSize; }

      public:
        void setData(const MukPathGraph&);
        using VisualObject::setData;

      private:
        double mPointSize;
    };

  }
}
