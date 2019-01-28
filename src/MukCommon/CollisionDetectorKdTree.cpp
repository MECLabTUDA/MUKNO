#include "private/muk.pch"
#include "CollisionDetectorKdTree.h"

#include "muk_common.h"
#include "MukException.h"

#include <vtkSmartPointer.h>
#include <vtkPolyData.h>

#pragma warning( push )
#pragma warning( disable : 4996 )
// used internally by cgal
#ifndef CGAL_EIGEN3_ENABLED
#define CGAL_EIGEN3_ENABLED
#endif

//#include <CGAL/Epick_d.h>
//#include <CGAL/Manhattan_distance_iso_box_point.h>
//#include <CGAL/Euclidean_distance_sphere_point.h>
//#include <CGAL/K_neighbor_search.h>
//#include <CGAL/Search_traits_d.h>
//#include <CGAL/Kd_tree.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/point_generators_2.h>
#include <CGAL/Orthogonal_k_neighbor_search.h>
#include <CGAL/Search_traits_3.h>
#pragma warning( pop )

#include <map>

namespace
{
  //typedef CGAL::Epick_d<CGAL::Dimension_tag<3> > Kernel;
  //typedef Kernel::Point_d     Point_d;
  //typedef Kernel::Iso_box_d   Iso_box_d;
  //typedef Kernel              TreeTraits;
  //typedef CGAL::Manhattan_distance_iso_box_point<TreeTraits> Distance;
  //typedef CGAL::K_neighbor_search<TreeTraits, Distance> Neighbor_search;
  //typedef Neighbor_search::Tree Tree;

  typedef CGAL::Simple_cartesian<double> K;
  typedef K::Point_3 Point_d;
  typedef CGAL::Search_traits_3<K> TreeTraits;
  typedef CGAL::Orthogonal_k_neighbor_search<TreeTraits> Neighbor_search;
  typedef Neighbor_search::Tree Tree;

  typedef std::unique_ptr<Tree> TreePointer;


  struct Obstacle
  {
    Obstacle() : active(true) {}
    ~Obstacle() {}

    bool        active;
    vtkSmartPointer<vtkPolyData> data;
  };
}

namespace gris
{
  namespace muk
  {
    struct CollisionDetectorKdTree::Impl
    {
      Impl() 
        : kdTree(std::make_unique<Tree>())
      {
      }
      ~Impl()
      {
        if (kdTree)
          kdTree->clear();
      }

      std::map<std::string, Obstacle> mapObstacles;
      mutable TreePointer kdTree;
    };

    // ---------------------------------------------------------------

    /**
    */
    CollisionDetectorKdTree::CollisionDetectorKdTree()
      : mp(std::make_unique<Impl>())
    {
    }

    /**
    */
    CollisionDetectorKdTree::~CollisionDetectorKdTree()
    {
    }

    /**
    */
    void CollisionDetectorKdTree::addObstacle(const std::string& key, vtkSmartPointer<vtkPolyData>& obj)
    {
      Obstacle o;
      o.data = obj;
      mp->mapObstacles.insert(std::make_pair(key, o));
      mOutOfDate = true;
    }

    /**
    */
    void CollisionDetectorKdTree::removeObstacle(const std::string& key)
    {
      mp->mapObstacles.erase(key);
      mOutOfDate = true;
    }

    /**
    */
    void CollisionDetectorKdTree::setActive(const std::string& key, bool b)
    {
      auto iter = mp->mapObstacles.find(key);
      if (iter==mp->mapObstacles.end())
        return;
      if (iter->second.active != b)
      {
        mOutOfDate = true;
        iter->second.active = b;
      }
    }

    /**
    */
    bool CollisionDetectorKdTree::hasKey(const std::string& key) const
    {
      auto iter = mp->mapObstacles.find(key);
      if (iter==mp->mapObstacles.end())
        return false;
      else
        return true;
    }

    /**
    */
    std::vector<std::string> CollisionDetectorKdTree::getKeys() const
    {
      std::vector<std::string> ret;
      for (const auto& pair : mp->mapObstacles)
      {
        ret.push_back(pair.first);
      }
      return ret;
    }

    /**
    */
    bool CollisionDetectorKdTree::isActive(const std::string& key) const
    {      
      if ( ! hasKey(key) )
        throw MUK_EXCEPTION("no obstacle available with this key", key.c_str());
      return mp->mapObstacles.at(key).active;
    }

    /**
    */
    bool CollisionDetectorKdTree::nearestNeighbor(const Vec3d& p, Vec3d& nn, double maxDist) const
    {
      Point_d upper(p.x(), p.y(), p.z());
      Point_d lower(p.x(), p.y(), p.z());
      /*Iso_box_d query(lower, upper);
      double eps = 0.0;
      
      Neighbor_search N1(*mp->kdTree, query, 1, eps, true);      
      if (N1.begin()==N1.end())
        return false;

      Point_d nearest = N1.begin()->first;
      nn = Vec3d(nearest.at(0), nearest.at(1), nearest.at(2));
      Distance dist;
      if ( dist.inverse_of_transformed_distance(N1.begin()->second) < maxDist*maxDist )
      {
        return true;
      }
      else
      {
        return false;
      }*/
      // Initialize the search structure, and search all N points
      Neighbor_search search(*mp->kdTree, upper, 1);
      // report the N nearest neighbors and their distance
      // This should sort all N points by increasing distance from origin
      auto iter = search.begin();
      if (iter != search.end())
      {
        nn = Vec3d(iter->first.x(), iter->first.y(), iter->first.z());
        return true;
      }
      else
      {
        nn = p;
        return false;
      }
    }

    /** 
    */    
    bool CollisionDetectorKdTree::hasNeighbors(const Vec3d& p, double maxDist) const
    {
      Point_d upper(p.x(), p.y(), p.z());
      Point_d lower(p.x(), p.y(), p.z());
      /*Iso_box_d query(lower, upper);
      double eps = 0.1;

      Neighbor_search N1(*mp->kdTree, query, 1, eps);
      if (N1.begin()==N1.end())
        return false;
      Distance dist;
      if ( dist.inverse_of_transformed_distance(N1.begin()->second) < maxDist*maxDist )
      {
        return true;
      }
      else
      {
        return false;
      }*/
      Neighbor_search search(*mp->kdTree, upper, 1);
      // report the N nearest neighbors and their distance
      // This should sort all N points by increasing distance from origin
      auto iter = search.begin();
      if (iter != search.end() && std::sqrt(iter->second) < maxDist )
      {
        return true;
      }
      else
      {
        return false;
      }
    }
    
    /**
    */
    void CollisionDetectorKdTree::rebuild() const
    {
      mp->kdTree->clear();
      for (const auto& obj : mp->mapObstacles)
      {
        if (obj.second.active)
        {
          vtkPoints* points = obj.second.data->GetPoints();
          const vtkIdType N = points->GetNumberOfPoints();
          double p[3];
          for (vtkIdType i(0); i<N; ++i)
          {
            points->GetPoint(i, p);
            mp->kdTree->insert(Point_d(p[0], p[1], p[2]));
          }
        }
      }
      mp->kdTree->build();
      mOutOfDate = false;
    }

    /**
    */
    void CollisionDetectorKdTree::clear()
    {
      mp->kdTree->clear();
      mp->mapObstacles.clear();
      mOutOfDate = true;
    }

  }
}
  