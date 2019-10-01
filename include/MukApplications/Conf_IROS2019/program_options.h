#pragma once

#include <gstd/XmlDocument.h>
#include <gstd/XmlNode.h>

namespace gris
{
  namespace muk
  {
    /**
    */
    struct ProgramInput
    {
      XmlNode root();

      //NeedleExpInput   needleInput;
      //OtobasisExpInput otobasisInput;
      //HeartExpInput    heartInput;
      std::string configFile = "../resources/MukApplications/Conf_IROS2019/Conf_IROS2019.mukcfg";
      std::unique_ptr<XmlDocument> pConfigDoc;
    };

    bool process_command_line(int argc, char ** argv, ProgramInput& programInput);
  }
}
