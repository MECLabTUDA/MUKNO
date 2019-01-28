#include "private/muk.pch"
#include "polygon_3d_tools.h"

#pragma warning (push)
#pragma warning (disable : 4996)
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Projection_traits_yz_3.h>
#include <CGAL/Polygon_2_algorithms.h>
#include <CGAL/convex_hull_2.h>
//#include <CGAL/Polygon_2.h>
#pragma warning (pop)

namespace
{
  typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
  typedef K::Point_3 Point_3;
}

namespace gris
{
  namespace muk
  {
    /**
    */
    struct Polygon3D::Impl
    {
      std::vector<Point_3> mPoints;
    };

    /**
    */
    Polygon3D::Polygon3D()
      : mp(std::make_unique<Impl>())
    {
    }

    /**
    */
    Polygon3D::~Polygon3D()
    {
    }

    /**
    */
    void Polygon3D::create(const std::vector<Vec3d>& v)
    {
      std::transform(v.begin(), v.end(), std::back_inserter(mp->mPoints), [&] (const auto& p) { return Point_3(p.x(), p.y(), p.z()); });
      std::vector<Point_3> res;
      CGAL::convex_hull_2(mp->mPoints.begin(), mp->mPoints.end(), back_inserter(res), CGAL::Projection_traits_yz_3<K>());
      mp->mPoints.swap(res);
    }

    /**
    */
    bool Polygon3D::isInside(const Vec3d& p)
    {
      const auto query = Point_3(p.x(), p.y(), p.z());
      auto en = CGAL::bounded_side_2(mp->mPoints.begin(), mp->mPoints.end(), query, CGAL::Projection_traits_yz_3<K>());
      switch(en)
      {
        case CGAL::ON_BOUNDED_SIDE :
          return true;
        case CGAL::ON_BOUNDARY:
          return true;
        case CGAL::ON_UNBOUNDED_SIDE:
          return false;
        default:
          return false;
      }
    }

    
    
  }
}