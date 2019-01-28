#pragma once
#include "ITKWrapper.h"


#include "private\LargestRegionSliceFilter.h"

namespace gris
{
	namespace muk
	{
		/*
		The output is a binary image with the largest connected component remaining
		*/
		class MUK_ALGO_API LargestRegionSlice : public AlgorithmWrapper
		{
		public:
			LargestRegionSlice();

		public:
			static  const char* s_name() { return "LargestRegionSlice"; };
			virtual const char* name()   const { return s_name(); }

		public:
			virtual void   setInput(unsigned int portId, void* pDataType);
			virtual void*  getOutput(unsigned int portId);
			virtual size_t sizeInputs()  const { return 1; }
			virtual size_t sizeOutputs() const { return 1; }
			virtual void   update();

		protected:
			itk::SmartPointer<LargestRegionSliceFilter> largestRegionFilter;
		};
	}
} // gris