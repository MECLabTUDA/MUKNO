#pragma once

#include "MukVisualization/muk_visualization_api.h"

namespace gris
{
  namespace muk
  {    
    enum MUK_VIS_API EnColors
    {
      black,
      blue,
      bone,
      brown,
      cyan,
      gold,
      green,
      gray,      
      magenta,
      melon,
      nerve,
      olive,
      orange,
      red,
      snow,
      white,
      yellow,
      N_DEFAULT_COLORS
    };

    class MUK_VIS_API Colors
    {
      public:
        static double* Get(EnColors c);

      public:
        static double Artery[3];
        static double Black[3];
        static double Blue[3];
        static double BlueishGreen[3];
        static double BlueishRed[3];
        static double Bone[3];
        static double Brain[3];
        static double Brown[3];
        static double Cyan[3];
        static double DarkGreen[3];
        static double Gold[3];
        static double Gray[3];
        static double Green[3];
        static double LightCyan[3];
        static double LightGray[3];
        static double LightMagenta[3];
        static double LightYellow[3];
        static double Magenta[3];
        static double MarineBlue[3];
        static double Melon[3];
        static double Nerve[3];
        static double Olive[3];
        static double Orange[3];
        static double Purple[3];
        static double Red[3];
        static double Snow[3];
        static double Vein[3];
        static double White[3];
        static double Yellow[3];
        static double YellowishGreen[3];
    };


  }
}