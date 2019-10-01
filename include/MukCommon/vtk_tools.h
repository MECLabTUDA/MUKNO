#pragma once

template <typename T>
class vtkSmartPointer;

#define TypeVtk(type) vtkSmartPointer<type>
#define DeclVtk(type) vtkSmartPointer<type>
#define DefVtk(type, name) vtkSmartPointer<type> name = vtkSmartPointer<type>::New()
#define NewVtk(type) vtkSmartPointer<type>::New()

namespace gris
{
  namespace muk
  {
    /** \brief Create a vtk shared pointer through the vtk mechanism

      vtk's factory does not support constructor arguments, so Args can actually never be used.
    */
    template <typename T, class... Args>
    inline vtkSmartPointer<T> make_vtk(Args&... args)
    {
      return vtkSmartPointer<T>::New(std::forward<Args>(args)...);
    }
  }
}