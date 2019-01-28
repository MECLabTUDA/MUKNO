#pragma once
#include "private/program_options.h"

namespace gris
{
  namespace muk
  {
    /** \brief a generic experiment
    */
    class Experiment
    {
      public:
        Experiment(const ExperimentInput& input, EnExperimentType type)
          : mInput(input)
          , mExpType(type)
        {
        }

      public:
        virtual void initialize();
        virtual void run();
        virtual void eval();
        bool skipPatient(const PatientData& patientData);

      protected:
        virtual void runExperiment(const PatientData& patientData) {}
        virtual void evalExperiment(const PatientData& patientData) {}  /// first eval indivuals patients
        virtual void evalExperiments() {}  /// then combine & summarize individual results

      protected:
        const ExperimentInput& mInput;
        EnExperimentType mExpType;
    };

    /**
    */
    class PreprocessingAlgorithm : public Experiment
    {
      public:
        PreprocessingAlgorithm(const AlgInfo& input, EnExperimentType type = enPreprocessingAlgo);

      public:
        virtual void initialize();
        virtual void runExperiment(const PatientData& patientData);

      private:
        const AlgInfo& mAlgInfo;
    };
  }
}