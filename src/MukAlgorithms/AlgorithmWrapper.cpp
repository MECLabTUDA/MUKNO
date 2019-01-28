#include "private/muk.pch"
#include "AlgorithmWrapper.h"

#include "MukCommon/muk_common.h"
#include "MukCommon/MukException.h"


namespace gris
{
namespace muk
{
  template<> auto* toDataType<enImageInt3D>(void* pData)        { return static_cast<ImageInt3D*>(pData); }
  template<> auto* toDataType<enImageFloat3D>(void* pData)   { return static_cast<ImageFloat3D*>(pData); }
  template<> auto* toDataType<enVtkImage>(void* pData)        { return static_cast<vtkImageData*>(pData); }
  template<> auto* toDataType<enVtkPolyData>(void* pData)     { return static_cast<vtkPolyData*>(pData); }

  /**
  */
  AlgorithmWrapper::AlgorithmWrapper()
  {
    declareProperty<EnDisplayType>("DisplayType", MUK_SET(EnDisplayType, setDisplayType), MUK_GET(getDisplayType));
    declareProperty<std::string>("Alias", MUK_SET(std::string, setAlias), MUK_GET(getAlias));
  }
}
}

namespace std
{
	std::ostream& operator<< (std::ostream& os, const gris::muk::EnDisplayType& obj)
	{
		const int i(obj);
		return os << gris::muk::DisplayTypeNames[i];
	}

	/**
	*/
	std::istream& operator>> (std::istream& is, gris::muk::EnDisplayType& obj)
	{
		std::string tmp;
		is >> tmp;
		auto begin = gris::muk::DisplayTypeNames.begin();
		auto end = gris::muk::DisplayTypeNames.end();
		auto iter = std::find_if(begin, end, [&](const auto& str) { return tmp == str; });
		if (iter == end)
		{
			LOG_LINE << "WARNING: could not interpret '" << tmp << "' as EnDisplayType";
		}
		else
		{
			obj = gris::muk::EnDisplayType(std::distance(begin, iter));
		}
		return is;
	}
}