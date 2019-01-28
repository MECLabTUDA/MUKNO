#pragma once
#include "muk_visualization_api.h"

#include "vtkPolyDataAlgorithm.h"

namespace gris
{
  namespace muk
  {
    /** \brief A highly customizable vtkSource for the anticipated drilling robot (Bohrmolch)
    */
    class MUK_VIS_API MukRobotSource : public vtkPolyDataAlgorithm
    {
      public:
        vtkTypeMacro(MukRobotSource, vtkPolyDataAlgorithm);
        static MukRobotSource* New();
        MukRobotSource(int res = 100);

        int RequestData(
          vtkInformation *vtkNotUsed(request),
          vtkInformationVector **vtkNotUsed(inputVector),
          vtkInformationVector *outputVector);

        int RequestInformation(vtkInformation *, vtkInformationVector **, vtkInformationVector *);

        // GET and SET for Center
        vtkSetVector3Macro(Center, double);
        vtkGetVectorMacro(Center, double, 3);

        // GET and SET for Angle
        vtkSetMacro(Angle, double);
        vtkGetMacro(Angle, double);

        // GET and SET for LargeCylinderHeight
        vtkSetMacro(LargeCylinderHeight, double);
        vtkGetMacro(LargeCylinderHeight, double);

        // GET and SET for LargeCylinderRadius
        vtkSetMacro(LargeCylinderRadius, double);
        vtkGetMacro(LargeCylinderRadius, double);

        // GET and SET for LargeCylinderRGB
        vtkSetVector3Macro(LargeCylinderRGB, float);
        vtkGetVectorMacro(LargeCylinderRGB, float, 3);

        // GET and SET for PadRadius
        vtkSetMacro(PadRadius, double);
        vtkGetMacro(PadRadius, double);

        // GET and SET for PadRGB
        vtkSetVector3Macro(PadRGB, float);
        vtkGetVectorMacro(PadRGB, float, 3);

        // GET and SET for HeadLength
        vtkSetMacro(HeadLength, double);
        vtkGetMacro(HeadLength, double);

        // GET and SET for HeadRGB
        vtkSetVector3Macro(HeadRGB, float);
        vtkGetVectorMacro(HeadRGB, float, 3);

        // GET and SET for EngineCylinderHeight
        vtkSetMacro(EngineCylinderHeight, double);
        vtkGetMacro(EngineCylinderHeight, double);

        // GET and SET for EngineCylinderRadius
        vtkSetMacro(EngineCylinderRadius, double);
        vtkGetMacro(EngineCylinderRadius, double);

        // GET and SET for  EngineRGB
        vtkSetVector3Macro(EngineRGB, float);
        vtkGetVectorMacro(EngineRGB, float, 3);

        // GET and SET for BellowCylinderHeight
        vtkSetMacro(BellowCylinderHeight, double);
        vtkGetMacro(BellowCylinderHeight, double);

        // GET and SET for BellowCylinderRadius
        vtkSetMacro(BellowCylinderRadius, double);
        vtkGetMacro(BellowCylinderRadius, double);

        // GET and SET for  BellowCylinderRGB
        vtkSetVector3Macro(BellowCylinderRGB, float);
        vtkGetVectorMacro(BellowCylinderRGB, float, 3);

        // GET and SET for BellowPointsN
        vtkSetMacro(BellowPointsN, int);
        vtkGetMacro(BellowPointsN, int);

      protected:
        double Center[3];
        int Resolution;
        double Angle;					// Winkel

        double LargeCylinderHeight;
        double LargeCylinderRadius;
        float LargeCylinderRGB[3];

        double PadRadius;				// Ausdehnungskissen
        float PadRGB[3];

        double HeadLength;				// Bohrkopf
        float HeadRGB[3];

        double EngineCylinderHeight;	// Antriebswelle
        double EngineCylinderRadius;
        float EngineRGB[3];

        double BellowCylinderHeight;	// Faltenbalg
        double BellowCylinderRadius;
        float BellowCylinderRGB[3];

        int BellowPointsN;
    };
  }
}