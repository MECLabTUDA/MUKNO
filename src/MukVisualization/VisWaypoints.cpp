#include "private/muk.pch"
#include "VisWaypoints.h"
#include "PolyDataHandler.h"
#include "PolyDataMapperHandler.h"

#include "MukCommon/muk_common.h"
#include "MukCommon/MukException.h"

#include "MukVisualization/muk_colors.h"

#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkPoints.h>
#include <vtkPolyDataMapper.h>
#include <vtkArrowSource.h>
#include <vtkMath.h>
#include <vtkMatrix4x4.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkAppendPolyData.h>
#include <vtkPointData.h>
#include <vtkUnsignedCharArray.h>
#include <vtkActor.h>

#include <boost/format.hpp>

namespace
{
  const double arbitrary[3] = { vtkMath::Random(-10, 10), vtkMath::Random(-10, 10), vtkMath::Random(-10, 10) };
}

namespace gris
{
  namespace muk
  {
    const char* VisWaypoints::s_topologyStr[N_Topologies] =
    {
      "ArrowView",
      "LineView",
    };

    /**
    */
    VisWaypoints::VisWaypoints(const Waypoints& wp)
      : VisualObject(wp)
      , mpDirections (make_vtk<vtkPolyData>())
      , mWaypoints(&wp)
      , changed(false)
    {
      mTopology = arrows;
      mpDatas.push_back(make_vtk<vtkPolyData>());
      mpMappers.push_back(make_vtk<vtkPolyDataMapper>());
      mpActors.push_back(make_vtk<vtkActor>());
      mpMappers.at(0)->SetInputData(mpDatas.at(0));
      mpActors.at(0)->SetMapper(mpMappers.at(0));

      setDefaultColor(Colors::Yellow);

      declareProperty<EnTopology>("Topology",
        std::bind(&VisWaypoints::setTopology, this, std::placeholders::_1), 
        std::bind(&VisWaypoints::getTopology, this));
    }

    /**
    */
    /**
    */
    VisWaypoints::~VisWaypoints()
    {
    }

    /**
    */
    std::vector<std::string> VisWaypoints::getTopologies()
    {
      std::vector<std::string> ret;
      for(int i(0); i<N_Topologies; ++i)
        ret.push_back(s_topologyStr[i]);
      return ret;
    }

    /**
    */
    void VisWaypoints::update()
    {
      setData(*mWaypoints);
    }

    /**
    */
    void VisWaypoints::setTopology(EnTopology val)
    {
      PolyDataHandler::clearTopology(mpData.Get());
      PolyDataHandler::clearTopology(mpDatas.at(0).Get());
      mTopology = val;      
      reloadTopology();
    }

    /**
    */
    void VisWaypoints::setData(const Waypoints& wp)
    {    
      mWaypoints = &wp;
      auto points  = make_vtk<vtkPoints>();
      auto pointsD = make_vtk<vtkPoints>();
      for (const auto& obj : mWaypoints->states())
      {
        points ->InsertNextPoint(obj.coords.data());
        pointsD->InsertNextPoint(obj.tangent.data());
      }
      mpData->SetPoints(points);
      mpDirections->SetPoints(pointsD);
      reloadTopology();
      changed = false;
    }

    /**
    */
    void VisWaypoints::setColors(const std::vector<Vec3d>& colors)
    {
      VisualObject::setColors(colors);
      const int N = mpData->GetNumberOfPoints();
      const int N_arrows = mpDatas.at(0)->GetNumberOfPoints();
      const int pointsPerArrow = N_arrows/N;
      std::vector<Vec3d> arrowColors(N_arrows);
      for (int i(0); i<N; ++i)
      {
        std::fill(arrowColors.begin() + i*pointsPerArrow, arrowColors.begin() + (i+1)*pointsPerArrow , colors[i]);
      }
      auto colorArray = make_vtk<vtkUnsignedCharArray>();
      colorArray->SetName("Colors");
      colorArray->SetNumberOfComponents(3);

      for (const auto& c : arrowColors)
      {
        char col[3] = {static_cast<char>(255 * c.x()), static_cast<char>(255 * c.y()), static_cast<char>(255 * c.z()) };
        colorArray->InsertNextTuple3(col[0], col[1], col[2]);
      }
      mpDatas.at(0)->GetPointData()->AddArray(colorArray);
      mpMappers.at(0)->SetColorModeToDefault(); // unsigned char scalars are treated as colors, and NOT mapped through the lookup table, while everything else is.
      mpMappers.at(0)->SetScalarModeToUsePointFieldData();
      mpMappers.at(0)->SelectColorArray("Colors");
      mpMappers.at(0)->ScalarVisibilityOn();
      mpMappers.at(0)->Update();
    }

    /**
    */
    /*Waypoints VisWaypoints::asWaypoints() const
    {
      Waypoints output;
      const size_t N = mpData->GetNumberOfPoints();
      output.states().clear();
      for (size_t i(0); i<N; ++i)
      {
        double pos[3];
        double dir[3];
        mpData->GetPoint(i, pos);
        mpDirections->GetPoint(i, dir);
        MukState nextState = MukState(Vec3d(pos), Vec3d(dir));
        output.insertPoint(nextState, i);
      }
      return output;
    }*/

    /**
    */
    void VisWaypoints::insertState(size_t idx, const Vec3d& point)
    {
      const size_t N = mpData->GetNumberOfPoints();
      if (N < idx)
        throw MUK_EXCEPTION_SIMPLE( (boost::format("idx (%d) larger than size (%d)!") % idx % N).str().c_str() );

      auto newData       = make_vtk<vtkPolyData>();
      auto newPositions  = make_vtk<vtkPoints>();
      auto newDirections = make_vtk<vtkPoints>();
      for(size_t i(0); i<idx; ++i)
      {
        double p[3];
        mpData->GetPoint(i, p);
        newPositions->InsertNextPoint(p);
        mpDirections->GetPoint(i, p);
        newDirections->InsertNextPoint(p);
      } 
      newPositions->InsertNextPoint(point.data());
      Vec3d pStart = idx == 0 ? point : Vec3d(mpData->GetPoint(idx-1));
      Vec3d pEnd = idx == N ? point : Vec3d(mpData->GetPoint(idx));
      pEnd -= pStart;
      pEnd.normalize();
      newDirections->InsertNextPoint(pEnd.data());
      for(size_t i(idx); i<N; ++i)
      {
        double p[3];
        mpData->GetPoint(i, p);
        newPositions->InsertNextPoint(p);
        mpDirections->GetPoint(i, p);
        newDirections->InsertNextPoint(p);
      }
      newData->SetPoints(newPositions);
      mpDirections->SetPoints(newDirections);
      setData(newData);
      reloadTopology();
      changed = true;
    }

    /**
    */
    void VisWaypoints::setPosition(size_t idx, const Vec3d& point)
    {
      if (mpData->GetNumberOfPoints() < (int)idx)
        throw MUK_EXCEPTION_SIMPLE( (boost::format("VisWaypoints::setPoint(): idx (%d) larger than size (%d)!") % idx % mpData->GetNumberOfPoints()).str().c_str() );
      double p[3];
      mpData->GetPoints()->GetPoint(idx, p);
      p[0] = point.x();
      p[1] = point.y();
      p[2] = point.z();
      mpData->GetPoints()->SetPoint(idx, p);
      reloadTopology();
      changed = true;
    }

    /**
    */
    void VisWaypoints::setDirection(size_t idx, const Vec3d& point)
    {
      if (mpDirections->GetNumberOfPoints() < (int)idx)
        throw MUK_EXCEPTION_SIMPLE( (boost::format("VisWaypoints::setPoint(): idx (%d) larger than size (%d)!") % idx % mpData->GetNumberOfPoints()).str().c_str() );
      double p[3];
      mpDirections->GetPoints()->GetPoint(idx, p);
      p[0] = point.x();
      p[1] = point.y();
      p[2] = point.z();
      mpDirections->GetPoints()->SetPoint(idx, p);
      reloadTopology();
      changed = true;
    }

    /**
    */
    void VisWaypoints::deleteState(size_t idx)
    {
      const size_t N = mpData->GetNumberOfPoints();
      if (N < idx)
        throw std::exception( (boost::format("VisWaypoints::setPoint(): idx (%d) larger than size (%d)!") % idx % N).str().c_str() );

      auto positions  = make_vtk<vtkPoints>();
      auto directions = make_vtk<vtkPoints>();
      for(size_t i(0); i<N; ++i)
      {
        if (i==idx)
          continue;
        double* p = mpData->GetPoint(i);
        positions->InsertNextPoint(p[0], p[1], p[2]);
        p = mpDirections->GetPoint(i);
        directions->InsertNextPoint(p);
      }
      mpData->SetPoints(positions);
      mpDirections->SetPoints(directions);
      setData(mpData);
      reloadTopology();
      changed = true;
    }

    /**
    */
    MukState VisWaypoints::getState(size_t idx)
    {
      if (mpData->GetNumberOfPoints()<(int)idx)
        throw MUK_EXCEPTION_SIMPLE( (boost::format("VisWaypoints::getPoint(): idx (%d) larger than size (%d)!") % idx % mpData->GetNumberOfPoints()).str().c_str() );          
      double p[3];
      mpData->GetPoint(idx, p);
      double d[3];
      mpDirections->GetPoint(idx, d);
      return MukState(Vec3d(p), Vec3d(d));
    }

    /**
    */
    void VisWaypoints::reloadTopology()
    {
      switch (mTopology)
      {
        case arrows:
        {
          auto dataArrow   = make_vtk<vtkPolyData>();
          auto arrowSource = make_vtk<vtkArrowSource>();
          {            
            auto appendFilter = make_vtk<vtkAppendPolyData>();
            const int N = mpData->GetNumberOfPoints();
            std::vector<vtkSmartPointer<vtkPolyData>> arrows(N);
            for (int idx(0); idx<N; ++idx)
            {
              // Generate start and end point
              double startPoint[3];
              double endPoint[3];
              for (int i(0); i!=3; ++i)
              {               
                startPoint[i] = mpData->GetPoint(idx)[i];
                endPoint[i]   = mpData->GetPoint(idx)[i] + mpDirections->GetPoint(idx)[i];
              }
              // Compute a basis
              double normalizedX[3];
              double normalizedY[3];
              double normalizedZ[3];
              // The X axis is a vector from start to end
              vtkMath::Subtract(endPoint, startPoint, normalizedX);
              double length = vtkMath::Norm(normalizedX);
              vtkMath::Normalize(normalizedX);
              // The Z axis is an arbitrary vector cross X
              vtkMath::Cross(normalizedX, arbitrary, normalizedZ);
              vtkMath::Normalize(normalizedZ);
              // The Y axis is Z cross X
              vtkMath::Cross(normalizedZ, normalizedX, normalizedY);
              auto matrix = make_vtk<vtkMatrix4x4>();
              // Create the direction cosine matrix
              matrix->Identity();
              for (unsigned int i = 0; i < 3; i++)
              {
                matrix->SetElement(i, 0, normalizedX[i]);
                matrix->SetElement(i, 1, normalizedY[i]);
                matrix->SetElement(i, 2, normalizedZ[i]);
              }
              // Apply the transforms
              auto transform = make_vtk<vtkTransform>();
              transform->Translate(startPoint);
              transform->Concatenate(matrix);
              transform->Scale(length, length, length);
              // Transform the polydata
              auto transformPD = make_vtk<vtkTransformPolyDataFilter>();
              transformPD->SetTransform(transform);
              transformPD->SetInputConnection(arrowSource->GetOutputPort());
              transformPD->Update();
              arrows[idx] = make_vtk<vtkPolyData>();
              arrows[idx]->DeepCopy(transformPD->GetOutput());
              appendFilter->AddInputData(arrows[idx]);
            }
            if (N>0)
            {
              appendFilter->Update();
              dataArrow = appendFilter->GetOutput();
            }
            dataArrow->GetPointData()->AddArray(mpDatas.at(0)->GetPointData()->GetArray("Colors"));
            mpDatas.at(0) = dataArrow;
            mpMappers.at(0)->SetInputData(mpDatas.at(0));
          }
          break;
        }
        case lines:
        {
          PolyDataHandler::addLines(mpData.Get());
          mpMapper->SetInputData(mpData);
          break;
        }
      }
      mpMappers.at(0)->Update();
      mpMapper->Update();
    }

  }
}


namespace std
{
  std::ostream& operator<< (std::ostream& os, const gris::muk::VisWaypoints::EnTopology& obj)
  {
    using namespace gris::muk;
    const int i (obj);    
    return os << VisWaypoints::s_topologyStr[i];
  }

  std::istream& operator>> (std::istream& is, gris::muk::VisWaypoints::EnTopology& obj)
  {
    using namespace gris::muk;
    std::string tmp;
    is >> tmp;
    auto begin = VisWaypoints::s_topologyStr;
    auto end   = VisWaypoints::s_topologyStr + VisWaypoints::EnTopology::N_Topologies;
    auto iter  = std::find_if(begin, end, [&] (const char* str) { return tmp == str; });
    if (iter == end)
    {
      LOG_LINE << "WARNING: could not interpret'" << tmp << "' as IInterpolator::EnInterpolationTypes";
    }
    else
    {
      obj = VisWaypoints::EnTopology(std::distance(begin, iter));
    }
    return is;
  }
}