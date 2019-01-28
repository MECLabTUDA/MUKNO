#include "private/muk.pch"
#include "SegmentationEvaluator.h"

#include "MukImaging\muk_imaging_tools.h"

#include <itkBinaryThresholdImageFilter.h>
#include <itkLabelOverlapMeasuresImageFilter.h>

namespace gris
{
namespace muk
{
  /**
  */
  SegmentationEvaluator::SegmentationEvaluator()
  {
  }

  /**
  */
  void SegmentationEvaluator::update()
  {
    auto thresh = make_itk < itk::BinaryThresholdImageFilter<ImageInt3D,ImageInt3D>>();
    thresh->SetInput(mpSegmentation);
    thresh->SetLowerThreshold(mLabel);
    thresh->SetUpperThreshold(mLabel);
    thresh->SetInsideValue(mLabel);
    thresh->SetOutsideValue(0);
    thresh->Update();
    auto filter = make_itk<itk::LabelOverlapMeasuresImageFilter<ImageInt3D>>();
    filter->SetSourceImage(mpGroundTruth);
    filter->SetTargetImage(thresh->GetOutput());
    filter->Update();
    mCurrentResult.dice    = filter->GetDiceCoefficient(mLabel);
    mCurrentResult.jaccard = filter->GetJaccardCoefficient(mLabel);
    mCurrentResult.falsePositiveError = filter->GetFalsePositiveError();
    mCurrentResult.falseNegativeError = filter->GetFalseNegativeError();

    auto measures = filter->GetLabelSetMeasures();
    auto labelMeasures = measures[mLabel];
    const auto TP = static_cast<double>(labelMeasures.m_Intersection);
    const auto FP = static_cast<double>(labelMeasures.m_Target - labelMeasures.m_Intersection);
    const auto FN = static_cast<double>(labelMeasures.m_Source - labelMeasures.m_Intersection);
    labelMeasures = measures[0];
    const auto TN = static_cast<double>(labelMeasures.m_Source);
    mCurrentResult.sensitivity = TP / (TP + FN);
    mCurrentResult.specificity = TN / (TN + FP);
  }

}
}