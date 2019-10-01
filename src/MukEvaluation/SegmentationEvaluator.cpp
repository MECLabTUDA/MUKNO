#include "private/muk.pch"
#include "SegmentationEvaluator.h"

#include "MukImaging\muk_imaging_tools.h"

#include <itkBinaryThresholdImageFilter.h>
#include <itkLabelOverlapMeasuresImageFilter.h>
#include <itkHausdorffDistanceImageFilter.h>

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
    auto threshGT = make_itk < itk::BinaryThresholdImageFilter<ImageInt3D,ImageInt3D>>();
    threshGT->SetInput(mpGroundTruth);
    threshGT->SetLowerThreshold(mLabel);
    threshGT->SetUpperThreshold(mLabel);
    threshGT->SetInsideValue(mLabel);
    threshGT->SetOutsideValue(0);
    threshGT->Update();
    auto threshSeg = make_itk < itk::BinaryThresholdImageFilter<ImageInt3D,ImageInt3D>>();
    threshSeg->SetInput(mpSegmentation);
    threshSeg->SetLowerThreshold(mLabel);
    threshSeg->SetUpperThreshold(mLabel);
    threshSeg->SetInsideValue(mLabel);
    threshSeg->SetOutsideValue(0);
    threshSeg->Update();
    {
      auto filter = make_itk<itk::LabelOverlapMeasuresImageFilter<ImageInt3D>>();    
      filter->SetSourceImage(threshGT->GetOutput());
      filter->SetTargetImage(threshSeg->GetOutput());
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
    {
      auto filter = make_itk<itk::HausdorffDistanceImageFilter<ImageInt3D, ImageInt3D>>();
      filter->SetInput1(threshGT->GetOutput());
      filter->SetInput2(threshSeg->GetOutput());
      filter->Update();
      mCurrentResult.hausdorffDist = filter->GetHausdorffDistance();
    }
  }
}
}