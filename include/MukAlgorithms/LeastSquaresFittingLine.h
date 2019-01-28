#pragma once
#include "ITKWrapper.h"


#include "private\LeastSquaresFittingLineFilter.h"

namespace gris
{
	namespace muk
	{
		/*
		
		*/
		class MUK_ALGO_API LeastSquaresFittingLine : public AlgorithmWrapper
		{
		public:
			LeastSquaresFittingLine();

		public:
			static  const char* s_name() { return "LeastSquaresFittingLine"; };
			virtual const char* name()   const { return s_name(); }

		public:
			virtual void   setInput(unsigned int portId, void* pDataType);
			virtual void*  getOutput(unsigned int portId);
			virtual size_t sizeInputs()  const { return 1; }
			virtual size_t sizeOutputs() const { return 1; }
			virtual void   update();

		protected:
			itk::SmartPointer<LeastSquaresFittingLineFilter> leastSquaresFittingLineFilter;
		};
	}
} // gris