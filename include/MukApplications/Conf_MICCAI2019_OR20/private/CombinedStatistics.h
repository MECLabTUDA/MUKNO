#pragma once

#include <vector>

#include <boost/filesystem.hpp>

namespace gris
{
  namespace muk
  {
    /**
    */
    struct CombinedStatistics
    {
      CombinedStatistics(size_t N);

      void write(int k, const boost::filesystem::path& dir);
      void writeFinal(int k, const boost::filesystem::path& dir);
      void read (int k, const boost::filesystem::path& dir);
      void resize(size_t n);

      std::vector<size_t> nGT;
      std::vector<size_t> nUnet;
      std::vector<size_t> nPasm;

      std::vector<std::vector<double>> minDistsGt;
      std::vector<std::vector<double>> minDistsUnet;
      std::vector<std::vector<double>> minDistsPasm;

      double radius;
      double dMin; // safetyDist
    };
  }
}