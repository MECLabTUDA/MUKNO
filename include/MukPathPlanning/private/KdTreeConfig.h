#pragma once

#include <memory>

#pragma warning( push )
#pragma warning( disable : 4996 )
// used internally by cgal
#ifndef CGAL_EIGEN3_ENABLED
#define CGAL_EIGEN3_ENABLED
#endif

#include <CGAL/Epick_d.h>
#include <CGAL/Manhattan_distance_iso_box_point.h>
#include <CGAL/Euclidean_distance_sphere_point.h>
#include <CGAL/K_neighbor_search.h>
#include <CGAL/Search_traits_d.h>
#include <CGAL/Kd_tree.h>

#pragma warning( pop )

namespace gris
{
  namespace muk
  {
    namespace collision
    {
      typedef CGAL::Epick_d<CGAL::Dimension_tag<3> > Kernel;
      typedef Kernel::Point_d     Point_d;
      typedef Kernel::Iso_box_d   Iso_box_d;
      typedef Kernel              TreeTraits;
      typedef CGAL::Manhattan_distance_iso_box_point<TreeTraits> Distance;
      typedef CGAL::K_neighbor_search<TreeTraits, Distance> Neighbor_search;
      typedef Neighbor_search::Tree Tree;
      
      typedef std::shared_ptr<Tree> TreePointer;
    }
  }
}
