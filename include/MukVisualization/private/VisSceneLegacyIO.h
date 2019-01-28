#pragma once

#include <string>

namespace gris
{
  class XmlNode;

  namespace muk
  {
    class VisScene;

    /**
    */
    class VisSceneLegacyIO
    {
    public:
      static bool load(const std::string& filename, const XmlNode& root, VisScene* dummyScene);
      static void load_v_0_4_1_0(const std::string& filename, const XmlNode& root, VisScene* dummyScene);
      static void load_v_0_5(const std::string& filename, const XmlNode& root, VisScene* dummyScene);
    };
  }
}