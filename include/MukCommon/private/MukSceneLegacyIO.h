#pragma once

#include <string>

namespace gris
{
  class XmlNode;

  namespace muk
  {    
    class MukScene;

    /**
    */
    class MukSceneLegacyIO
    {
      public:
        static bool load(const std::string& filename, const XmlNode& root, MukScene* dummyScene);
        static bool load_v_0_4_1_0(const std::string& filename, const XmlNode& root, MukScene* dummyScene);
    };
  }
}
