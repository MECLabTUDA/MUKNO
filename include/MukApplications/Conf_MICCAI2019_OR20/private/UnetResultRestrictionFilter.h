#pragma once
#include "MukAlgorithms/AlgorithmWrapper.h"

#include <memory>

namespace gris
{
  namespace muk
  {
    /** \brief Same Filter as in MukOtobasisSegmentation, just do not want to use the same version or link against some Otobasis stuff
    */
    class UnetResultRestrictionFilter : public AlgorithmWrapper
    {
      public:
        UnetResultRestrictionFilter();
        virtual ~UnetResultRestrictionFilter();

      public:
        static  const char* s_name()        { return "UnetResultRestrictionFilter"; }
        virtual const char* name()   const  { return s_name(); }

        virtual void   setInput (unsigned int portId, void* pDataType);
        virtual void*  getOutput(unsigned int portId);
        virtual size_t sizeInputs()  const { return 1u; }
        virtual size_t sizeOutputs() const { return 1u; }
        virtual void   update();

      private:
        struct Impl;
        std::unique_ptr<Impl> mp;
    };
  }
}
