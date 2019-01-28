#include "private/muk.pch"

#include "mukIO.h"
#include "muk_common.h"
#include "MukException.h"

#include <vtkGenericDataObjectReader.h>
#include <vtkGenericDataObjectWriter.h>
#include <vtkOBJReader.h>
#include <vtkSTLReader.h>
#include <vtkSTLWriter.h>
#include <vtkMarchingCubes.h>
#include <vtkPolyDataWriter.h>
#include <vtkMetaImageReader.h>

#include <itkMetaImageIOFactory.h>
#include <itkPNGImageIOFactory.h>
#include <itkImageFileReader.h>
#include <itkImageFileWriter.h>
#include <itkImage.h>
#include <itkImageToVTKImageFilter.h>
#include <itkMinimumMaximumImageCalculator.h>
#include <itkThresholdImageFilter.h>
#include <itkImageToHistogramFilter.h>

#include <vtkPointData.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkTriangle.h>

#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/serialization/vector.hpp>

#include <numeric>
#include <iostream>



namespace
{
	namespace fs = boost::filesystem;
}

namespace gris
{
namespace muk
{
  /**
  */
	vtkSmartPointer<vtkPolyData> loadVtkFile(const std::string& filename)
	{
		fs::path path = filename;
		if ( ! fs::is_regular_file(path))
		{
			throw MUK_EXCEPTION("File does not exist", filename.c_str());
		}
    auto pGdoReader = make_vtk<vtkGenericDataObjectReader>();
    pGdoReader->SetFileName(path.string().c_str());
    pGdoReader->Update();
    if (!pGdoReader->IsFilePolyData())
		{
			throw MUK_EXCEPTION_SIMPLE("polydata as input needed!\n");
		}
    return pGdoReader->GetPolyDataOutput();
	}

  /**
  */
	vtkSmartPointer<vtkPolyData> loadMhdFile(const std::string& filename)
	{
		vtkSmartPointer<vtkPolyData> output;

		typedef unsigned short              PixelType;
		typedef itk::Image<PixelType, 3>    Image;
		typedef itk::ImageFileReader<Image> Reader;

		namespace fs = boost::filesystem;
		fs::path path = filename;
		if (0 != path.extension().string().compare(".mhd"))
		{
			throw MUK_EXCEPTION_SIMPLE("Input-file does not have ending 'mhd'!\n");
		}

		Reader::Pointer reader = Reader::New();
		reader->SetFileName(path.string());
		reader->Update();

		// Use VTK's Marching Cube to extract iso-surface
    auto pCubes = make_vtk<vtkMarchingCubes>();
		typedef itk::Image<unsigned short, 3> Image;
		typedef itk::ImageToVTKImageFilter<Image> Filter;
		Filter::Pointer pImg = Filter::New();
		pImg->SetInput(reader->GetOutput());
		pImg->Update();
		vtkImageData* pImgData = pImg->GetOutput();
		pCubes->SetInputData(pImg->GetOutput());
		pCubes->SetValue(0, 1);
		pCubes->Update();
		output = pCubes->GetOutput();
		return output;
	}

	std::vector<vtkSmartPointer<vtkPolyData>> loadSegmentedMhd(const std::string& filename)
	{
		fs::path path = filename;
		if (0 != path.extension().string().compare(".mhd"))
		{
			throw MUK_EXCEPTION_SIMPLE("Input-file does not have ending 'mhd'!\n");
		}

		typedef unsigned short              PixelType;
		typedef itk::Image<PixelType, 3>    Image;
		typedef itk::ImageFileReader<Image> Reader;

		Reader::Pointer reader = Reader::New();
		reader->SetFileName(path.string());
		reader->Update();

		// compute maximum
		typedef itk::Statistics::ImageToHistogramFilter<Image> HistFilter;
		HistFilter::HistogramType::Pointer histogram;
		{
			HistFilter::Pointer histFilter = HistFilter::New();
			histFilter->SetInput(reader->GetOutput());
			const unsigned int MeasurementVectorSize = 1;   // Grayscale
			const unsigned int binsPerDimension = 256; // number of bins
			HistFilter::HistogramType::MeasurementVectorType lowerBound(MeasurementVectorSize); // important in generic cases or if automated computation of min/max is turned off?
			HistFilter::HistogramType::MeasurementVectorType upperBound(MeasurementVectorSize);
			lowerBound.Fill(0);
			upperBound.Fill(256);
			HistFilter::HistogramType::SizeType size(MeasurementVectorSize);
			size.Fill(binsPerDimension);
			histFilter->SetMarginalScale(1);
			histFilter->SetHistogramBinMinimum(lowerBound);
			histFilter->SetHistogramBinMaximum(upperBound);
			histFilter->SetHistogramSize(size);
			histFilter->SetAutoMinimumMaximum(false); // this is ridiculously important. if true, the filter computes minimum and maximum and devides the bin values from this min and max
			histFilter->Update();
			histogram = histFilter->GetOutput();
		}

		// compute each region beginning with value 1
		std::vector<vtkSmartPointer<vtkPolyData>> output;

		for (unsigned int i = 1; i < histogram->GetSize()[0]; ++i)
		{
			unsigned int value = histogram->GetFrequency(i);
			if (value != 0)
			{
				typedef itk::ThresholdImageFilter<Image> ThresholdFilter;
				ThresholdFilter::Pointer thresholdFilter = ThresholdFilter::New();
				thresholdFilter->SetInput(reader->GetOutput());
				thresholdFilter->ThresholdOutside(i, i);
				thresholdFilter->SetOutsideValue(0);
				thresholdFilter->Update();

				// Use VTK's Marching Cube to extract iso-surface
				// note that this algo isn implemented in ITK
				vtkSmartPointer<vtkMarchingCubes> pCubes = vtkSmartPointer<vtkMarchingCubes>::New();
				typedef itk::ImageToVTKImageFilter<Image> ToVtkFilter;
				ToVtkFilter::Pointer vtkFilter = ToVtkFilter::New();
				vtkFilter->SetInput(thresholdFilter->GetOutput());
				vtkFilter->Update();
				pCubes->SetInputData(vtkFilter->GetOutput());
				pCubes->SetValue(0, 1);
				pCubes->Update();
				output.push_back(pCubes->GetOutput());
			}
		}
		return output;
	}

	/**
	*/
	vtkSmartPointer<vtkPolyData> loadObjFile(const std::string& filename)
	{
		fs::path path = filename;
		if (0 != path.extension().string().compare(".obj"))
		{
			auto str = boost::format("%s: Wrong file type: '%s'. Could not read file: '%s'") % __FUNCTION__ %  path.extension() % path.string();
			throw MUK_EXCEPTION_SIMPLE(str.str().c_str());
		}
    auto pReader = make_vtk<vtkOBJReader>();
    pReader->SetFileName(filename.c_str());
    pReader->Update();
    auto pResult = make_vtk<vtkPolyData>();
    pResult = pReader->GetOutput();
    return pResult;
	}

	/**
	*/
	vtkSmartPointer<vtkPolyData> loadStlFile(const std::string& filename)
	{
		fs::path path = filename;
    if ( (! fs::is_regular_file(path))
      || ! (path.extension().string() == ".STL" || path.extension().string() == ".stl"))
		{
			auto str = boost::format("%s: Wrong file type: '%s'. Could not read file: '%s'") % __FUNCTION__ %  path.extension() % path.string();
			throw MUK_EXCEPTION_SIMPLE(str.str().c_str());
		}
    auto pReader = make_vtk<vtkSTLReader>();
    pReader->SetFileName(filename.c_str());
    pReader->Update();
    auto pResult = make_vtk<vtkPolyData>();
    pResult = pReader->GetOutput();
    return pResult;
	}

	/**
	*/
	void writeSTL(const vtkSmartPointer<vtkPolyData> data, const char* filename)
	{
		fs::path fn(filename);
		if (0 != strcmp(".stl", fn.extension().string().c_str()))
		{
			throw MUK_EXCEPTION("File needs extension .stl", fn.string().c_str());
		}
    auto pWriter = make_vtk<vtkSTLWriter>();
    pWriter->SetInputData(data);
    pWriter->SetFileName(fn.string().c_str());
    pWriter->Update();
    pWriter->Write();
	}

	/** \brief calls one of the implemented specialized methods
	*/
	std::vector<vtkSmartPointer<vtkPolyData>> loadAnyFile(const std::string& filename)
	{
		fs::path path = filename;

		if (!fs::is_regular_file(path))
		{
			auto str = boost::format("%s: File does not exist: '%s'") % __FUNCTION__ % path.string();
			throw MUK_EXCEPTION_SIMPLE(str.str().c_str());
		}

		if (0 == path.extension().string().compare(".mhd"))
		{
			return loadSegmentedMhd(filename);
		}
		if (0 == path.extension().string().compare(".vtk"))
		{
			std::vector<vtkSmartPointer<vtkPolyData>> result;
			auto obj = loadVtkFile(filename);
			result.push_back(obj);
			return result;
		}
		if (0 == path.extension().string().compare(".obj"))
		{
			std::vector<vtkSmartPointer<vtkPolyData>> result;
			auto obj = loadObjFile(filename);
			result.push_back(obj);
			return result;
		}
		if (0 == path.extension().string().compare(".stl")
			|| 0 == path.extension().string().compare(".STL"))
		{
			std::vector<vtkSmartPointer<vtkPolyData>> result;
			auto obj = loadStlFile(filename);
			result.push_back(obj);
			return result;
		}

		auto str = boost::format("%s: File-Type %s not supported! Could not read '%s'") % __FUNCTION__ % path.extension() % path.string();
		throw MUK_EXCEPTION_SIMPLE(str.str().c_str());
	}

	/**
	*/
	void saveMukPath(const MukPath& p, const char* filename)
	{
		auto fn = boost::filesystem::path(filename);
		std::ofstream ofs(fn.string());
		boost::archive::text_oarchive oa(ofs);
		oa << p;
	}

	/**
	*/
	void saveMukPathAsTxt(const MukPath& p, const char* filename)
	{
		auto fn = boost::filesystem::path(filename);
		std::ofstream ofs(fn.string());
		ofs << p.getRadius() << std::endl;
		for (const auto& state : p.getPath())
		{
			ofs << state.coords << " " << state.tangent << std::endl;
		}
	}

	/**
	*/
	MukPath loadMukPath(const std::string& filename)
	{
		boost::filesystem::path fn = filename;
		if (!fs::is_regular_file(fn))
		{
			throw MUK_EXCEPTION("File does not exist", fn.string().c_str());
		}
		MukPath path;
		std::ifstream ifs(fn.string());
		boost::archive::text_iarchive ia(ifs);
		ia >> path;
		return path;
	}

	/**
	*/
	MukPath loadMukPathFromTxt(const std::string& filename)
	{
		boost::filesystem::path fn = filename;
		if (!fs::is_regular_file(fn))
		{
			throw MUK_EXCEPTION("File does not exist", fn.string().c_str());
		}
		MukPath path;
		std::ifstream ifs(fn.string());
		std::string line;
		{
			std::getline(ifs, line);
			path.setRadius(std::stof(line.c_str()));
		}
		std::vector<MukState> states;
		while (std::getline(ifs, line))
		{
			std::stringstream ss(line);
			MukState next;
			ss >> next.coords >> next.tangent;
			states.push_back(next);
		}
    LOG_LINE << "size " << states.size();
		path.setPath(states);
		return path;
	}

  /**
  */
  void saveMukPathGraph(const MukPathGraph& g, const std::string& filename)
  {
    std::ofstream ofs(filename.c_str());
    g.print(ofs);
  }

  /**
  */
  MukPathGraph loadMukPathGraph(const std::string& filename)
  {
    std::ifstream ifs(filename.c_str());
    MukPathGraph graph;
    graph.read(ifs);
    return graph;
  }

	/**
	*/
	void saveToVtkFile(const vtkSmartPointer<vtkPolyData> data, const char* filename)
	{
		fs::path fn(filename);
		if (0 != strcmp(".vtk", fn.extension().string().c_str()))
		{
			throw MUK_EXCEPTION("File needs extension .vtk", fn.string().c_str());
		}
    auto pWriter = make_vtk<vtkPolyDataWriter>();
    pWriter->SetInputData(data);
    pWriter->SetFileName(fn.string().c_str());
    pWriter->Update();
    pWriter->Write();
	}

  /**
  */
  void saveVtkPolyData(const vtkSmartPointer<vtkPolyData> data, const std::string& filename)
  {
    auto fn = fs::path(filename);
    if ( ! fn.has_extension() )
    {
      throw MUK_EXCEPTION("File does not have an extension (.xxx). The vtkGenericObjWriter needs one to determine the type of file you want to write!", fn.string().c_str());
    }
    std::string extstr = fn.extension().string();
    std::vector<std::string> possibleEndings = { ".vtk", ".obj", ".stl" };
    if (std::none_of(possibleEndings.begin(), possibleEndings.end(), [&] (const auto& str) { return str == extstr; }))
    {
      std::stringstream ss;
      ss << "Possible extensions:";
      for(const auto& str : possibleEndings)
        ss << " " << str;
      throw MUK_EXCEPTION((std::string("File does not have a valid extension!" ) + ss.str()).c_str(), fn.string().c_str());
    }

    auto pWriter = make_vtk<vtkGenericDataObjectWriter>();
    pWriter->SetInputData(data);
    pWriter->SetFileName(filename.c_str());
    pWriter->Update();
    pWriter->Write();
	}

	/** \brief loads an .off-file (corresnpondence file of the pdmlib. 
      
    Does not check for file existence!
    Will throw if ifstream fails somewhere during reading
  */
  MUK_COMMON_API vtkSmartPointer<vtkPolyData> loadOffFile(const std::string& filename)
  {
    std::ifstream ifs(filename);
    std::string line;
    std::getline(ifs, line);
    // todo OFF means no normals, NOFF means normals in pdmlib io. ATM, only OFF is supported
    if (line != "OFF")
      throw MUK_EXCEPTION("no off file", filename.c_str());
      
    std::getline(ifs, line);
    int N_V, N_F, N_E; // number of vertices, en´dges, facs
    {
      std::stringstream ss(line);
      ss >> N_V >> N_F >> N_E;
    }
    auto points = make_vtk<vtkPoints>();
    for (int i(0); i<N_V; ++i)
    {
      std::getline(ifs, line);
      std::stringstream ss(line);
      double x, y, z;
      ss >> x >> y >> z;
      points->InsertNextPoint(x, y, z);
    }
    auto triangles = make_vtk<vtkCellArray>();
    for (int i(0); i<N_F; ++i)
    {
      auto triangle = make_vtk<vtkTriangle>();
      std::getline(ifs, line);
      std::stringstream ss(line);
      int nInts, id1, id2, id3;
      ss >> nInts >> id1 >> id2 >> id3;
      triangle->GetPointIds()->SetId ( 0, id1 );
      triangle->GetPointIds()->SetId ( 1, id2 );
      triangle->GetPointIds()->SetId ( 2, id3 );
      triangles->InsertNextCell(triangle);
    }
    auto polyData = make_vtk<vtkPolyData>();
    polyData->SetPoints(points);
    polyData->SetPolys(triangles);
    return polyData;
  }
}
}