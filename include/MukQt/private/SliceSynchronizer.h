#pragma once

class vtkRenderWindowInteractor;
class vtkRenderer;

#include <functional>

namespace gris
{
  namespace muk
  {
    /**
    */
    class SliceSynchronizer
    {
      public:
        struct Group
        {
          int id;
          vtkRenderWindowInteractor* interactor;
          vtkRenderer* renderer;
        };

      public:
        SliceSynchronizer();
        ~SliceSynchronizer();

      public:
        void addRenderGroup(const Group& style);
        void synchronize();

      private:
        struct Impl;
        std::unique_ptr<Impl> mp;
    };
  }
}