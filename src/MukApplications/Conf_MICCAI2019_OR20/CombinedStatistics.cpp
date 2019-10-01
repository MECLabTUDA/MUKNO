#include "private/muk.pch"
#include "private/CombinedStatistics.h"

#include "private/custom_streams.h"

#include <numeric>

namespace gris
{
namespace muk
{
  /**
  */
  CombinedStatistics::CombinedStatistics(size_t n) 
  {
    resize(n);
  }

  /** \brief
  */
  void CombinedStatistics::resize(size_t n)
  {
    nGT.resize(n);
    nUnet.resize(n);
    nPasm.resize(n);
    minDistsGt.resize(n);
    minDistsUnet.resize(n);
    minDistsPasm.resize(n);
  }

  /**
  */
  void CombinedStatistics::write(int k, const boost::filesystem::path& dir)
  {
    using namespace gris::io;
    const auto fn = dir / "stats.txt";
    auto ofs = std::ofstream(fn.string());
    ofs << "Size " << std::endl;
    ofs << nGT.size() << std::endl;
    ofs << "GT" << std::endl;
    ofs << nGT << std::endl;
    for (const auto& dists : minDistsGt)
    {
      ofs << dists << std::endl;
    }
    ofs << "Unet" << std::endl;
    ofs << nUnet << std::endl;
    for (const auto& dists : minDistsUnet)
    {
      ofs << dists << std::endl;
    }
    ofs << "Pasm" << std::endl;
    ofs << nPasm << std::endl;
    for (const auto& dists : minDistsPasm)
    {
      ofs << dists << std::endl;
    }
  }

  /** \brief
  */
  void CombinedStatistics::read(int k, const boost::filesystem::path& dir)
  {
    using namespace gris::io;
    const auto fn = dir / "stats.txt";
    auto ifs = std::ifstream(fn.string());
    std::string nextLine;
    std::string dummy;
    // read in size
    std::getline(ifs, dummy);
    std::getline(ifs, nextLine);
    size_t N;
    {
      auto ss = std::stringstream(nextLine);
      ss >> N;
      resize(N);
    }
    nGT.clear();
    nUnet.clear();
    nPasm.clear();
    // read GT results
    std::getline(ifs, dummy);
    std::getline(ifs, nextLine);
    {
      auto ss = std::stringstream(nextLine);
      ss >> nGT;
    }
    for (size_t i(0); i<N ; ++i)
    {
      std::getline(ifs, nextLine);
      auto ss = std::stringstream(nextLine);
      for (size_t j(0); j< nGT[i]; ++j)
      {
        double d;
        ss >> d;
        minDistsGt[i].push_back(d);
      }
    }
    // read Unet results
    std::getline(ifs, dummy);
    std::getline(ifs, nextLine);
    {
      auto ss = std::stringstream(nextLine);
      ss >> nUnet;
    }
    for (size_t i(0); i<N ; ++i)
    {
      std::getline(ifs, nextLine);
      auto ss = std::stringstream(nextLine);
      for (size_t j(0); j< nUnet[i]; ++j)
      {
        double d;
        ss >> d;
        minDistsUnet[i].push_back(d);
      }
    }
    // read Pasm results
    std::getline(ifs, dummy);
    std::getline(ifs, nextLine);
    {
      auto ss = std::stringstream(nextLine);
      ss >> nPasm;
    }
    for (size_t i(0); i<N ; ++i)
    {
      std::getline(ifs, nextLine);
      auto ss = std::stringstream(nextLine);
      for (size_t j(0); j< nPasm[i]; ++j)
      {
        double d;
        ss >> d;
        minDistsPasm[i].push_back(d);
      }
    }
  }

  /** \brief
  */
  void CombinedStatistics::writeFinal(int k, const boost::filesystem::path& dir)
  {
    const auto N = nGT.size();
    const auto N_GT   = std::count_if(nGT.begin(), nGT.end(), [&](auto n) { return n > 0; });
    const auto N_Unet = std::count_if(nUnet.begin(), nUnet.end(), [&](auto n) { return n > 0; });
    const auto N_Pasm = std::count_if(nPasm.begin(), nPasm.end(), [&](auto n) { return n > 0; });
    
    const auto successRateGT   = N_GT * 1.0 / N;
    const auto successRateUnet = N_Unet * 1.0 / N;
    const auto successRatePasm = N_Pasm * 1.0 / N;
    auto l_mean = [&](const std::vector<double>& v) { return v.empty() ? 0.0 : std::accumulate(v.begin(), v.end(), 0.0) / (1.0*v.size()); };    
    auto meanGT = std::accumulate(minDistsGt.begin(), minDistsGt.end(), 0.0,       [&] (double in, const auto& v) { return in + radius + l_mean(v); }) / (1.0*N_GT);
    auto meanUnet = std::accumulate(minDistsUnet.begin(), minDistsUnet.end(), 0.0, [&] (double in, const auto& v) { return in + radius + l_mean(v); }) / (1.0*N_Unet);
    auto meanPasm = std::accumulate(minDistsPasm.begin(), minDistsPasm.end(), 0.0, [&] (double in, const auto& v) { return in + radius + l_mean(v); }) / (1.0*N_Pasm);
    meanGT   = N_GT   ? meanGT : 0.0;
    meanUnet = N_Unet ? meanUnet : 0.0;
    meanPasm = N_Pasm ? meanPasm : 0.0;
    auto l_fail = [&] (const std::vector<double>& v) { return std::none_of(v.begin(), v.end(), [&] (double d) { return d > dMin; }); };
    auto failUnet = std::count_if(minDistsUnet.begin(), minDistsUnet.end(), [&] (const auto& v) { return ! v.empty() && l_fail(v); }) / (1.0*N_Unet);
    auto failPasm = std::count_if(minDistsPasm.begin(), minDistsPasm.end(), [&] (const auto& v) { return ! v.empty() && l_fail(v); }) / (1.0*N_Pasm);
    failUnet = N_Unet ? failUnet : 0.0;
    failPasm = N_Pasm ? failPasm : 0.0;

    using namespace gris::io;
    const auto fn = dir / "stats_table.txt";
    auto ofs = std::ofstream(fn.string());
    ofs << "success rate" << std::endl;
    ofs << "GT    " << successRateGT << std::endl;
    ofs << "U-Net " << successRateUnet << std::endl;
    ofs << "Ours  " << successRatePasm << std::endl;
    ofs << std::endl;
    ofs << "mean safety dist" << std::endl;
    {
      std::string strGt = N_GT  ? std::to_string(meanGT) : "-";
      std::string strUnet = N_Unet ? std::to_string(meanUnet) : "-";
      std::string strPasm = N_Pasm ? std::to_string(meanPasm) : "-";
      ofs << "GT    " << strGt << std::endl;
      ofs << "U-Net " << strUnet << std::endl;
      ofs << "Ours  " << strPasm << std::endl;
      ofs << std::endl;
    }
    ofs << "failure rate" << std::endl;
    {
      std::string strUnet = N_Unet ? std::to_string(failUnet) : "-";
      std::string strPasm = N_Pasm ? std::to_string(failPasm) : "-";
      ofs << "GT    " << "-" << std::endl;
      ofs << "U-Net " << strUnet << std::endl;
      ofs << "Ours  " << strPasm << std::endl;
    }
  }
}
}