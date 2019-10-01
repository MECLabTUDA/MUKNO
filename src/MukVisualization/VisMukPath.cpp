#include "private/muk.pch"
#include "VisMukPath.h"
#include "PolyDataHandler.h"
#include "PolyDataMapperHandler.h"
#include "muk_colors.h"

#include "MukCommon/vtk_tools.h"

#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkCellArray.h>
#include <vtkPoints.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderer.h>
#include <vtkActor.h>
#include <vtkProperty.h>
#include <vtkPointData.h>
#include <vtkFloatArray.h>
#include <vtkCleanPolyData.h>

#include <boost/format.hpp>

namespace
{
}

namespace gris
{
  namespace muk
  {
    const char* VisMukPath::s_topologyStr[N_Topologies] =
    {
      "PointView",
      "LineView",
      "TubeView",
    };

    /**
    */
    VisMukPath::VisMukPath(const MukPath& pPath)
      : VisualObject(pPath)
      , mPath(pPath)
      , mLineWidth(1.0)
    {
      mTopology = Lines;
      mDefaultColor = Vec3d(Colors::Green);
      setColors(mDefaultColor);
      declareProperty<EnTopology>("Topology"
        , [&] (const EnTopology type) { this->setTopology(type); }
        , [&] ()                      { return this->getTopology(); });
      declareProperty<double>("LineWidth"
        , [&] (double type) { setLineWidth(type); }
        , [&] ()            { return getLineWidth(); });
    }

    /**
    */
    VisMukPath::~VisMukPath()
    {
    }

    /**
    */
    std::vector<std::string> VisMukPath::getTopologies()
    {
      std::vector<std::string> ret;
      for(int i(0); i<N_Topologies; ++i)
        ret.push_back(s_topologyStr[i]);
      return ret;
    }

    /**
    */
    void VisMukPath::update()
    {
      setData(mPath);
    }
        
    /**
    */
    void VisMukPath::setData(const MukPath& path)
    {
      auto data   = make_vtk<vtkPolyData>();
      auto points = make_vtk<vtkPoints>();
      for(const auto& obj : path.getStates())
      {
        points->InsertNextPoint(obj.coords.x(), obj.coords.y(), obj.coords.z());
      }
      data->SetPoints(points);
      setData(data);
      reloadTopology();
    }

    /**
    */
    MukPath VisMukPath::asMukPath()
    {      
      const int N = mpData->GetNumberOfPoints();
      std::vector<MukState> states;
      states.reserve(N);
      for(int i(0); i<N; ++i)
      {
        double d[3];
        double t1[3];
        double t2[3];
        mpData->GetPoint(i, d);
        if ((i != 0) && (i != N-1))
        {
          mpData->GetPoint(i - 1, t1);
          mpData->GetPoint(i + 1, t2);
          states.push_back(MukState(Vec3d(d), Vec3d(t2[0] - t1[0], t2[1] - t1[1], t2[2] - t1[2]).normalized()));
        }
        else
        {
          if (i == 0)
          {
            mpData->GetPoint(i + 1, t2);
            states.push_back(MukState(Vec3d(d), Vec3d(t2[0] - d[0], t2[1] - d[1], t2[2] - d[2]).normalized()));
          }
          else
          {
            mpData->GetPoint(i - 1, t1);
            states.push_back(MukState(Vec3d(d), Vec3d(d[0] - t1[0], d[1] - t1[1], d[2] - t1[2]).normalized()));
          }
        }
      }
      for(int i(1); i<N; ++i)
      {
        states[i].tangent = (states[i].coords - states[i-1].coords).normalized();
      }
      if(states.size() > 1)
        states[0].tangent = (states[1].coords - states[0].coords).normalized();

      
      MukPath path;
      path.setRadius(mPath.getRadius());
      path.setStates(states);
      return path;
    }

    /**
    */
    void VisMukPath::setTopology(EnTopology val)
    {
      mTopology = val;
      reloadTopology();
    }

    /**
    */
    void VisMukPath::setLineWidth(double d)
    {
      mLineWidth = d;
      reloadTopology();
    }

    /**
    */
    double VisMukPath::getLineWidth() const
    {
      return mLineWidth;
    }
    
    /**
    */
    void VisMukPath::reloadTopology()
    {
      PolyDataHandler::clearTopology(mpData.Get());
      switch (mTopology)
      {
        case Vertices:
        {
          PolyDataHandler::addVertices(mpData.Get());
          mpMapper->SetInputData(mpData);
          break;
        }
        case Lines:        
        {
          PolyDataHandler::addLines(mpData.Get(), mLineWidth);
          mpMapper->SetInputData(mpData);
          mpActor->GetProperty()->SetLineWidth(mLineWidth);
          mpActor->Modified();
          break;
        }
        case 
          Tube:
        {
          PolyDataHandler::addLines(mpData.Get(), mLineWidth);
          mpMapper->SetInputData(mpData);
          PolyDataMapperHandler::addTube(mpMapper.Get(), mPath.getRadius(), 20);
          break;
        }
        default:
        {
          LOG_LINE << "VisMukPath::setTopology() -> Topology unkown! (" << mTopology;
        }
      }
      mpMapper->Modified();
    }

  }
}

namespace std
{
  std::ostream& operator<< (std::ostream& os, const gris::muk::VisMukPath::EnTopology& obj)
  {
    using namespace gris::muk;
    const int i (obj);    
    return os << VisMukPath::s_topologyStr[i];
  }

  std::istream& operator>> (std::istream& is, gris::muk::VisMukPath::EnTopology& obj)
  {
    using namespace gris::muk;
    std::string tmp;
    is >> tmp;
    auto begin = VisMukPath::s_topologyStr;
    auto end   = VisMukPath::s_topologyStr + VisMukPath::EnTopology::N_Topologies;
    auto iter  = std::find_if(begin, end, [&] (const char* str) { return tmp == str; });
    if (iter == end)
    {
      LOG_LINE << "WARNING: could not interpret '" << tmp << "' as IInterpolator::EnInterpolationTypes";
    }
    else
    {
      obj = VisMukPath::EnTopology(std::distance(begin, iter));
    }
    return is;
  }
}