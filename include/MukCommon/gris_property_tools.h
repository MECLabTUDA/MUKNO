#pragma once
#include "muk_common_api.h"

#include <gstd/DynamicProperty.h>
#include <gstd/XmlNode.h>

namespace gris
{
  /** \brief Provides convenient way to insert/retrieve information with sanity operations for checking node existence or inserting missing ones.
  */
  struct MUK_COMMON_API XmlNodeHelper
  {
    static void write(XmlNode& node,       const std::string& key, const gstd::DynamicProperty& prop);
    static void read(const XmlNode& node, const std::string& key, gstd::DynamicProperty& prop);
  };
}
