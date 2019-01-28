#include "private/muk.pch"

#include "muk_colors.h"

#include <exception>

namespace gris
{
  namespace muk
  {
    // all with ones
    double Colors::White[3]   = {1, 1, 1};
    double Colors::Red[3]     = {1, 0, 0};
    double Colors::Green[3]   = {0, 1, 0};
    double Colors::Blue[3]    = {0, 0, 1};
    double Colors::Yellow[3]  = {1, 1, 0};
    double Colors::Cyan[3]    = {0, 1, 1};
    double Colors::Magenta[3] = {1, 0, 1};
    double Colors::Black[3]   = {0, 0, 0};

    // gray tones
    double Colors::LightGray[3]  = {0.9, 0.9, 0.9};
    double Colors::Gray[3]    = {0.5, 0.5, 0.5};

    // all with 1 and 0.5
    double Colors::YellowishGreen[3] = {0.5, 1.0, 0.0};
    double Colors::BlueishGreen[3]   = {0.0, 1.0, 0.5};
    double Colors::Orange[3]         = {1.0, 0.5, 0.0};
    double Colors::BlueishRed[3]     = {1.0, 0.0, 0.5};
    double Colors::Purple[3]         = {0.5, 0.0, 1.0};    
    double Colors::MarineBlue[3]     = {0.0, 0.5, 1.0};
    double Colors::LightYellow[3]    = {1.0, 1.0, 0.5};
    double Colors::LightMagenta[3]   = {1.0, 0.5, 1.0};
    double Colors::LightCyan[3]      = {0.5, 1.0, 1.0};

    // old weird ones
    double Colors::Artery[3]  = {136.0/255.0, 0, 0};
    double Colors::Bone[3]    = {0.66, 0.66, 0.5};
    double Colors::Brain[3]   = {1.0, 235/255.0, 205/255.0};
    double Colors::Brown[3]   = {0.55, 0.25, 0.05};
    double Colors::DarkGreen[3] = {0, 100/255.0, 0};
    double Colors::Gold[3]    = {1, 0.85, 0};
    double Colors::Melon[3]   = {0.85, 0.65, 0.4};
    double Colors::Nerve[3]   = {1.0, 1.0, 0.5};
    double Colors::Olive[3]   = {0.5, 0.5, 0};
    double Colors::Snow[3]    = {0.95, 0.9, 0.9};
    double Colors::Vein[3]    = {0, 0, 136.0/255.0};
    
    
    double* Colors::Get(EnColors c)
    {
      switch (c)
      {
        case black: return Black;
        case blue: return Blue;
        case bone: return Bone;
        case brown: return Brown;
        case cyan: return Cyan;
        case gold: return Gold;
        case gray: return Gray;
        case green: return Green;
        case magenta: return Magenta;
        case melon: return Melon;
        case nerve: return Nerve;
        case olive: return Olive;
        case orange: return Orange;
        case red : return Red;
        case snow: return Snow;
        case yellow: return Yellow;
        case white: return White;        
        default:
          throw std::exception("Color net implemented!");
      }
    }

  }
}