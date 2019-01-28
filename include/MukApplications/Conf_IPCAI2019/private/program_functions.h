#pragma once
#include "private/program_options.h"

#include <iostream>
#include <vector>

namespace gris
{
  namespace muk
  {
    std::vector<std::string> getDataSet1();
    std::vector<std::string> getDataSet2();
    void createPatientDataFile();
    void preprocess(const ProgramInput& input, const AlgInfo& alginfo);
    void saveEvaluationDate(const std::string& file, EnExperimentType en, const std::string& date, const std::string& alias);
    void saveEvaluationDateAlgo(const std::string& file, EnExperimentType en, const std::string& date, const std::string& alias);

    void extractObstacles(const ProgramInput& programInput);
    void evaluateStatistics();

    class MukScene;
    class SelectionModel;

    void evalDistances(MukScene& scene, SelectionModel& select, const std::string& canal, int idx, std::ofstream& ofs);
    std::string toAbbreviation(const std::string& key);
  }
}
