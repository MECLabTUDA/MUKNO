#include "private/muk.pch"
#include "MukImagingIO.h"

#include "MukCommon/MukException.h"

#include <itkImageFileReader.h>
#include <itkGDCMImageIO.h>
#include <itkGDCMSeriesFileNames.h>
#include <itkImageSeriesReader.h>
#include "itkCastImageFilter.h"
#include "itkAddImageFilter.h"
#include "itkClampImageFilter.h"
#include "itkMetaImageIOFactory.h"

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>

namespace
{
  namespace fs = boost::filesystem;
}

namespace gris
{
namespace muk
{
  /**

    from https://itk.org/ITKExamples/src/IO/DICOM/DicomSeriesReadImageWrite3/Documentation.html
    check magic values!
  */
  ImageInt3D::Pointer loadDicom(const std::string& dirName)
  {
    if ( ! fs::is_directory(dirName))
    {
      throw MUK_EXCEPTION("Directory does not exist!", dirName.c_str());
    }
    
    LOG_LINE << "analysing dicom directory. This may take a while";
    auto pNameGenerator = make_itk<itk::GDCMSeriesFileNames>();
    pNameGenerator ->SetUseSeriesDetails(true);
    pNameGenerator ->AddSeriesRestriction("0008|0021"); // that is the DICOM tag "SeriesDate"
    pNameGenerator ->SetGlobalWarningDisplay(false);
    pNameGenerator ->SetDirectory(dirName);
    
    {
      typedef std::vector< std::string >    SeriesIdContainer;
      const SeriesIdContainer& seriesUID = pNameGenerator->GetSeriesUIDs();
      SeriesIdContainer::const_iterator seriesItr = seriesUID.begin();
      SeriesIdContainer::const_iterator seriesEnd = seriesUID.end();

      if (seriesItr != seriesEnd)
      {
        LOG << "The directory: \n"
          << dirName << " contains the following DICOM Series: \n";
      }
      else
      {
        throw MUK_EXCEPTION("No DICOMs in folder!", dirName.c_str());
      }

      while (seriesItr != seriesEnd)
      {
        LOG << seriesItr->c_str() << "\n";
        ++seriesItr;
      }

      seriesItr = seriesUID.begin();
      if (seriesItr != seriesUID.end())
      {
        std::string seriesIdentifier;
        //if (false) // If seriesIdentifier given convert only that
        //{
        //  //seriesIdentifier = argv[3];
        //  seriesItr = seriesUID.end();
        //}
        //else //otherwise convert everything
        {
          seriesIdentifier = seriesItr->c_str();
          seriesItr++;
        }
        std::cout << "\nReading: ";
        std::cout << seriesIdentifier << std::endl;
        std::vector< std::string > fileNames;
        fileNames = pNameGenerator->GetFileNames(seriesIdentifier);

        auto pReader  = make_itk< itk::ImageSeriesReader<ImageInt3D> >();
        auto pDicomIO = make_itk<itk::GDCMImageIO>();
        pDicomIO->SetFileName(fileNames.front().c_str());
        pReader->SetImageIO(pDicomIO);
        pReader->SetFileNames(fileNames);
        pReader->Update();
        return pReader->GetOutput();
      }
      else
      {
        throw MUK_EXCEPTION("No DICOMs in folder!", dirName.c_str());
      }
    }
/*
    auto reader = make_itk< itk::ImageFileReader<ImageInt3DSeries::Image> >();
    reader->SetFileName( dirName.c_str() );
    

    auto gdcmImageIO = make_itk<itk::GDCMImageIO>();
    reader->SetImageIO( gdcmImageIO );
    reader->Update();
    return reader->GetOutput();*/
  }

  /**
  */
  ImageInt3D::Pointer loadMhd(const std::string& filename)
  {
	  if (!fs::is_regular_file(filename))
	  {
		  throw MUK_EXCEPTION("File does not exist!", filename.c_str());
	  }
	  const fs::path path = filename;
	  if (path.extension().string() != ".mhd" && path.extension().string() != ".mha")
	  {
		  throw MUK_EXCEPTION("Input-file needs file extension 'mhd' or 'mha'!", filename.c_str());
	  }
    
    auto reader = make_itk<itk::ImageFileReader<ImageInt3D>>();
    reader->SetFileName(path.string());
    reader->Update();
    return reader->GetOutput();

  }

  /**
  */
  ImageInt3D::Pointer loadMha(const std::string& filename)
  {
	  if (!fs::is_regular_file(filename))
	  {
		  throw MUK_EXCEPTION("File does not exist!", filename.c_str());
	  }
	  const fs::path path = filename;
	  if (0 != path.extension().string().compare(".mha"))
	  {
		  throw MUK_EXCEPTION("Input-file needs file extension 'mha'!", filename.c_str());
	  }

	  auto pReader = make_itk< itk::ImageFileReader<ImageInt3D> >();
	  pReader->SetFileName(filename.c_str());
	  pReader->Update();
	  return pReader->GetOutput();
  }
}
}