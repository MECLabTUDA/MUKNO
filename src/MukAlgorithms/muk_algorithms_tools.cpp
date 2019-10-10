#include "private/muk.pch"
#include "muk_algorithms_tools.h"

#include "MukCommon/muk_common.h"
#include "MukCommon/vtk_tools.h"

#include "MukImaging/muk_imaging_tools.h"

#include <itkTriangleCell.h>
#include <itkQuadrilateralCell.h>

#include <vtkPoints.h>
#include <vtkCellArray.h>

namespace
{
  using namespace gris::muk;

  /**
  */
  class VisitVTKCellsClass
  {
    vtkCellArray* m_Cells;
    int* m_LastCell;
    int* m_TypeArray;
    public:
    // typedef the itk cells we are interested in
    using CellInterfaceType = itk::CellInterface<TriangleMesh::PixelType, TriangleMesh::CellTraits>;

    typedef itk::LineCell<CellInterfaceType> floatLineCell;
    typedef itk::TriangleCell<CellInterfaceType>      floatTriangleCell;
    typedef itk::QuadrilateralCell<CellInterfaceType> floatQuadrilateralCell;

    // Set the vtkCellArray that will be constructed
    void SetCellArray(vtkCellArray* a)
    {
      m_Cells = a;
    }

    // Set the cell counter pointer
    void SetCellCounter(int* i)
    {
      m_LastCell = i;
    }

    // Set the type array for storing the vtk cell types
    void SetTypeArray(int* i)
    {
      m_TypeArray = i;
    }

    // Visit a triangle and create the VTK_TRIANGLE cell
    void Visit(unsigned long, floatTriangleCell* t)
    {
      m_Cells->InsertNextCell(3,  (vtkIdType*)t->PointIdsBegin());
      m_TypeArray[*m_LastCell] = VTK_TRIANGLE;
      (*m_LastCell)++;
    }

    // Visit a triangle and create the VTK_QUAD cell
    void Visit(unsigned long, floatQuadrilateralCell* t)
    {
      m_Cells->InsertNextCell(4,  (vtkIdType*)t->PointIdsBegin());
      m_TypeArray[*m_LastCell] = VTK_QUAD;
      (*m_LastCell)++;
    }

    // Visit a line and create the VTK_LINE cell
    void Visit(unsigned long, floatLineCell* t)
    {
      m_Cells->InsertNextCell(2,  (vtkIdType*)t->PointIdsBegin());
      m_TypeArray[*m_LastCell] = VTK_LINE;
      (*m_LastCell)++;
    }
  };
}

namespace gris
{
namespace muk
{
  /**
  */
  void polyDataToItkMesh(const vtkPolyData& poly_, TriangleMesh& mesh_)
  {
    auto* poly = const_cast<vtkPolyData*>(&poly_);
    auto* mesh = &mesh_;
    // first points
    const auto N = poly->GetNumberOfPoints();
    auto* vtkpoints = poly->GetPoints();
    mesh->GetPoints()->Reserve(N);
    for (vtkIdType i(0); i<N; ++i)
    {
      auto* p = vtkpoints->GetPoint(i);
      itk::Point<MukMeshPrecision, Dim3D> q;
      for(int j(0); j<Dim3D; ++j)
        q[j] = static_cast<MukMeshPrecision>(p[j]);
      mesh->SetPoint(i, q);
      //LOG_LINE << q;
    }
    // then triangles
    unsigned int numberOfTriangles = 0;
    // first count the number of triangles ???
    vtkIdType* cellPoints;
    vtkIdType  numberOfCellPoints;
    auto* triangleStrips = poly->GetStrips();
    triangleStrips->InitTraversal();
    while (triangleStrips->GetNextCell(numberOfCellPoints, cellPoints))
    {
      numberOfTriangles += numberOfCellPoints-2;
    }
    auto* polygons = poly->GetPolys();
    polygons->InitTraversal();
    while (polygons->GetNextCell(numberOfCellPoints, cellPoints))
    {
      if (numberOfCellPoints == 3)
      {
        ++numberOfTriangles;
      }
    }
    // Now copy the triangles from vtkPolyData into the itk::Mesh
    mesh->GetCells()->Reserve( numberOfTriangles );
    typedef TriangleMesh::CellType   CellType;
    typedef itk::TriangleCell< CellType > TriangleCellType;
    int cellId = 0;
    // first copy the triangle strips
    triangleStrips->InitTraversal();
    while( triangleStrips->GetNextCell( numberOfCellPoints, cellPoints ) )
    {
      unsigned int numberOfTrianglesInStrip = numberOfCellPoints - 2;
      unsigned long pointIds[3];
      pointIds[0] = cellPoints[0];
      pointIds[1] = cellPoints[1];
      pointIds[2] = cellPoints[2];
      for( unsigned int t=0; t < numberOfTrianglesInStrip; ++t )
      {
        TriangleMesh::CellAutoPointer c;
        TriangleCellType * tcell = new TriangleCellType;
        TriangleCellType::PointIdentifier itkPts[3];
        for (int ii = 0; ii < 3; ++ii)
        {
          itkPts[ii] = static_cast<TriangleCellType::PointIdentifier>(pointIds[ii]);
        }
        tcell->SetPointIds( itkPts );
        c.TakeOwnership( tcell );
        mesh->SetCell( cellId, c );
        ++cellId;
        pointIds[0] = pointIds[1];
        pointIds[1] = pointIds[2];
        pointIds[2] = cellPoints[t+3];
      }
    }
    // what the fuck??
    // then copy the normal triangles
    polygons->InitTraversal();
    while( polygons->GetNextCell( numberOfCellPoints, cellPoints ) )
    {
      if( numberOfCellPoints !=3 ) // skip any non-triangle.
      {
        continue;
      }
      TriangleMesh::CellAutoPointer c;
      TriangleCellType * t = new TriangleCellType;
      TriangleCellType::PointIdentifier itkPts[3];
      for (int ii = 0; ii < numberOfCellPoints; ++ii)
      {
        itkPts[ii] = static_cast<TriangleCellType::PointIdentifier>(cellPoints[ii]);
      }
      t->SetPointIds( itkPts );
      c.TakeOwnership( t ); // What. The. Fuck!!!!!
      mesh->SetCell( cellId, c );
      ++cellId;
    }
    /*LOG_LINE << __FUNCTION__;
    for (int i(0); i<mesh->GetNumberOfCells(); ++i)
    {
      TriangleMesh::CellAutoPointer c;
      mesh->GetCell(i, c);
      auto* ids = c->GetPointIds();
      auto N = c->GetNumberOfPoints();
      LOG << "   " << N << " ids:   ";
      for (size_t i(0); i<N; ++i)
        LOG << ids[i] << " ";
      LOG_LINE;
    }*/
    std::cout << "Number of Points =   " << mesh->GetNumberOfPoints() << std::endl;
    std::cout << "Number of Cells  =   " << mesh->GetNumberOfCells()  << std::endl;
  }

  /** \brief

    from https://itk.org/Wiki/ITK/Examples/Meshes/ConvertToVTK
    that code is a nightmare..
  */
  void itkMeshToVtkPolyData(const TriangleMesh& mesh_, vtkPolyData& poly_)
  {
    auto* mesh = &mesh_;
    auto* poly = &poly_;
    poly->Initialize(); // remove everything: points + topology
    // Get the number of points in the mesh
    int numPoints = mesh->GetNumberOfPoints();
    if(numPoints == 0)
    {
      auto p = make_vtk<vtkPoints>();
      auto t = make_vtk<vtkCellArray>();
      poly->SetPoints(p);
      poly->SetPolys(t);
      return;
    }

    auto vpoints = make_vtk<vtkPoints>();
    {
      vpoints->SetNumberOfPoints(numPoints);
      auto points = mesh->GetPoints();
      // In ITK the point container is not necessarily a vector, but in VTK it is
      vtkIdType VTKId = 0;
      std::map< vtkIdType, int > IndexMap;
      for (auto i = points->Begin(); i != points->End(); ++i, ++VTKId)
      {
        IndexMap[VTKId] = i->Index();
        // Set the vtk point at the index with the the coord array from itk
        // itk returns a const pointer, but vtk is not const correct, so
        // we have to use a const cast to get rid of the const
        vpoints->SetPoint(VTKId, const_cast<MukMeshPrecision*>(i->Value().GetDataPointer()));
      }
      poly->SetPoints(vpoints);
    }
    // Setup some VTK things
    int vtkCellCount = 0; // running counter for current cell being inserted into vtk
    int numCells = mesh->GetNumberOfCells();
    int *types = new int[numCells]; // type array for vtk
    auto cells = make_vtk<vtkCellArray>();
    cells->EstimateSize(numCells, 4);

    // Setup the line visitor
    /*using LineVisitor = itk::CellInterfaceVisitorImplementation<MukMeshPrecision, TriangleMesh::CellTraits,
      itk::LineCell< itk::CellInterface<TriangleMesh::PixelType, TriangleMesh::CellTraits > >, VisitVTKCellsClass>;
    auto lv = make_itk<LineVisitor>();
    lv->SetTypeArray(types);
    lv->SetCellCounter(&vtkCellCount);
    lv->SetCellArray(cells);*/

    // Setup the triangle visitor
    using TriangleVisitor = itk::CellInterfaceVisitorImplementation<MukMeshPrecision, TriangleMesh::CellTraits,
      itk::TriangleCell< itk::CellInterface<TriangleMesh::PixelType, TriangleMesh::CellTraits > >, VisitVTKCellsClass>;
    auto tv = make_itk<TriangleVisitor>();
    tv->SetTypeArray(types);
    tv->SetCellCounter(&vtkCellCount);
    tv->SetCellArray(cells);

    //// Setup the quadrilateral visitor
    /*using QuadrilateralVisitor = itk::CellInterfaceVisitorImplementation<MukMeshPrecision, TriangleMesh::CellTraits,    
      itk::QuadrilateralCell< itk::CellInterface<TriangleMesh::PixelType, TriangleMesh::CellTraits > >, VisitVTKCellsClass>;
    auto qv = make_itk<QuadrilateralVisitor>();
    qv->SetTypeArray(types);
    qv->SetCellCounter(&vtkCellCount);
    qv->SetCellArray(cells);*/

    // Add the visitors to a multivisitor
    auto mv = make_itk<TriangleMesh::CellType::MultiVisitor>();
    mv->AddVisitor(tv);
    /*mv->AddVisitor(qv);
    mv->AddVisitor(lv);*/
    // Now ask the mesh to accept the multivisitor which
    // will Call Visit for each cell in the mesh that matches the
    // cell types of the visitors added to the MultiVisitor
    mesh->Accept(mv);

    // Now set the cells on the vtk grid with the type array and cell array
    //poly->SetCells(types, cells);
    poly->SetPolys(cells);
    std::cout << "Unstructured grid has " << poly->GetNumberOfPoints() << " points." << std::endl;;
    std::cout << "Unstructured grid has " << poly->GetNumberOfCells() << " cells." << std::endl;;
  }
}
}