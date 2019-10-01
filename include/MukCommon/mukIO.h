#pragma once

#include "muk_common_api.h"
#include "MukPath.h"
#include "MukPathGraph.h"
#include "SystemCalibration.h"
#include "vtk_tools.h"

class vtkPolyData;

#include <string>
#include <vector>

namespace gris
{
	namespace muk
	{
		MUK_COMMON_API vtkSmartPointer<vtkPolyData> loadVtkFile(const std::string& filename);
		MUK_COMMON_API vtkSmartPointer<vtkPolyData> loadMhdFile(const std::string& filename);
		MUK_COMMON_API vtkSmartPointer<vtkPolyData> loadObjFile(const std::string& filename);
		MUK_COMMON_API vtkSmartPointer<vtkPolyData> loadStlFile(const std::string& filename);
    MUK_COMMON_API vtkSmartPointer<vtkPolyData> loadOffFile(const std::string& filename);
		MUK_COMMON_API std::vector<vtkSmartPointer<vtkPolyData>> loadSegmentedMhd(const std::string& filename);
		MUK_COMMON_API std::vector<vtkSmartPointer<vtkPolyData>> loadAnyFile(const std::string& filename);

		MUK_COMMON_API void saveToVtkFile(const vtkSmartPointer<vtkPolyData> data, const char* filename);
    MUK_COMMON_API void saveVtkPolyData(const vtkSmartPointer<vtkPolyData> data, const std::string& filename);
		MUK_COMMON_API void writeSTL(const vtkSmartPointer<vtkPolyData> data, const char* filename);

		MUK_COMMON_API void    saveMukPath(const MukPath& p, const char* filename);
		MUK_COMMON_API void    saveMukPathAsTxt(const MukPath& p, const char* filename);
		MUK_COMMON_API MukPath loadMukPath(const std::string& filename);
		MUK_COMMON_API MukPath loadMukPathFromTxt(const std::string& filename);

    MUK_COMMON_API void         saveMukPathGraph(const MukPathGraph& g, const std::string& filename);
    MUK_COMMON_API MukPathGraph loadMukPathGraph(const std::string& filename);
		

		
		
	}
}
