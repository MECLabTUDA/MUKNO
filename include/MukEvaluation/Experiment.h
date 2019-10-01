#pragma once

#include "muk_evaluation_api.h"

#include <gstd/DynamicProperty.h>
#include <gstd/XmlNode.h>

namespace gris
{
  namespace muk
  {
    /** \brief base class for clearly defined evaluation experiments, e.g. within a paper submission.
        
        supports the common need for an efficient reevaluation procedure based on repeated preprocessing, experiment run and subsequent evaluation.
        supports skipping of data sets for rapid prototyping / testing.
    */
    class MUK_EVAL_API Experiment
    {
      public:
        Experiment();
        virtual ~Experiment() {}

      public:
        virtual void initialize(const XmlNode& node);
        virtual void run() {}
        virtual void evaluate() {}
        virtual void finalize(XmlNode& node);
        virtual void print();

      public:
        bool skipId(unsigned int id);
      
      protected:
        std::string mName; /// to be set by a deriving class
        gstd::DynamicProperty mProps;
        // flags what to run
        bool mRun;         /// run experiment?        
        bool mEvaluate;    /// evaluate experiment ?
        // skip some data sets?
        std::vector<unsigned int> mSkipPatientsEqual;
        std::vector<unsigned int> mSkipPatientsUnequal;
        int mSkipPatientsHigher;  /// -1 indicates no skip
        int mSkipPatientsLesser; /// -1 indicates no skip
        
        std::string mLastEvalDate;  /// this will be overwritten in the config file if an experiment or algorithm has been run
        std::string mOutputRootDir; /// root output dir
        std::string mResourceDir;
        std::string mSubDir;
    };
  }
}
