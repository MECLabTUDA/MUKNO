#pragma once


namespace gris
{
  class XmlNode;

  namespace muk
  {    
    class VisScene;
    class MukScene;
    class VisualizationModel;

    /**
    */
    class VisualizationModelLegacyIO
    {
      public:
      static bool load(const XmlNode& root, MukScene* mpScene, VisScene* dummyScene, VisualizationModel* pModel);

      static bool load_v_0_0_0_0(const XmlNode& root, MukScene* mpScene, VisScene* dummyScene, VisualizationModel* pModel);
      static bool load_v_0_2_0_0(const XmlNode& root, MukScene* mpScene, VisScene* dummyScene, VisualizationModel* pModel);
      static bool load_v_0_3_0_0(const XmlNode& root, MukScene* mpScene, VisScene* dummyScene, VisualizationModel* pModel);
    };
  }
}