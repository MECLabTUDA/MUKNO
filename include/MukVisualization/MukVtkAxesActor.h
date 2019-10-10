#pragma once
#include "muk_visualization_api.h"

#include "vtkRenderingAnnotationModule.h" // For export macro
#include "vtkProp3D.h"
#include "vtkSmartPointer.h"

class vtkActor;
class vtkConeSource;
class vtkCylinderSource;
class vtkLineSource;
class vtkPolyData;
class vtkPropCollection;
class vtkProperty;
class vtkRenderer;
class vtkSphereSource;

/**
*/
class MUK_VIS_API MukVtkAxesActor : public vtkProp3D
{
  public:
    static MukVtkAxesActor *New();

    MukVtkAxesActor(const MukVtkAxesActor&) = delete;
    void operator=(const MukVtkAxesActor&)  = delete;

  protected:
    MukVtkAxesActor();
    ~MukVtkAxesActor();

  public:
    vtkTypeMacro(MukVtkAxesActor,vtkProp3D);
    void PrintSelf(ostream& os, vtkIndent indent);

  public:
    virtual void GetActors(vtkPropCollection *);
    virtual int RenderOpaqueGeometry(vtkViewport *viewport);
    virtual int RenderTranslucentPolygonalGeometry(vtkViewport *viewport);
    virtual int HasTranslucentPolygonalGeometry();
    virtual double*       GetBounds();
    virtual vtkMTimeType  GetRedrawMTime();

  public:
    // these are interfaces too, but not virtual
    void ShallowCopy(vtkProp *prop);
    void ReleaseGraphicsResources(vtkWindow *);
    void GetBounds(double bounds[6]);
    vtkMTimeType GetMTime();

  public:
    void SetTotalLength( double x, double y, double z );
    void SetTotalLength( double v[3] )                              { SetTotalLength( v[0], v[1], v[2] ); }
    vtkGetVectorMacro( mTotalLength, double, 3 );

    void SetNormalizedShaftLength( double x, double y, double z );
    void SetNormalizedShaftLength( double v[3] )                    { SetNormalizedShaftLength( v[0], v[1], v[2] ); }
    vtkGetVectorMacro( mNormalizedShaftLength, double, 3 );

    void SetNormalizedTipLength( double x, double y, double z );
    void SetNormalizedTipLength( double v[3] )                      { SetNormalizedTipLength( v[0], v[1], v[2] ); }
    vtkGetVectorMacro( mNormalizedTipLength, double, 3 );

    void SetNormalizedLabelPosition( double x, double y, double z );
    void SetNormalizedLabelPosition( double v[3] )                  { SetNormalizedLabelPosition( v[0], v[1], v[2] ); }
    vtkGetVectorMacro( mNormalizedLabelPosition, double, 3 );


    /** \brief Set/get the resolution of the pieces of the axes actor. */
    vtkSetClampMacro(mConeResolution, int, 3, 128);
    vtkGetMacro     (mConeResolution, int);
    vtkSetClampMacro(mSphereResolution, int, 3, 128);
    vtkGetMacro     (mSphereResolution, int);
    vtkSetClampMacro(mCylinderResolution, int, 3, 128);
    vtkGetMacro     (mCylinderResolution, int);

    /** \brief Set/get the radius of the pieces of the axes actor.    */
    vtkSetClampMacro(mConeRadius, double, 0, VTK_FLOAT_MAX);
    vtkGetMacro     (mConeRadius, double);
    vtkSetClampMacro(mSphereRadius, double, 0, VTK_FLOAT_MAX);
    vtkGetMacro     (mSphereRadius, double);
    vtkSetClampMacro(mCylinderRadius, double, 0, VTK_FLOAT_MAX);
    vtkGetMacro     (mCylinderRadius, double);

    /** \brief Set the type of the shaft to a cylinder, line, or user defined geometry.    */
    void SetShaftType( int type );
    void SetShaftTypeToCylinder()     { SetShaftType( MukVtkAxesActor::CYLINDER_SHAFT ); }
    void SetShaftTypeToLine()         { SetShaftType( MukVtkAxesActor::LINE_SHAFT ); }
    vtkGetMacro(mShaftType, int);

    /** \brief Set the type of the tip to a cone, sphere, or user defined geometry.    */
    void SetTipType( int type );
    void SetTipTypeToCone()           { SetTipType( MukVtkAxesActor::CONE_TIP ); }
    void SetTipTypeToSphere()         { SetTipType( MukVtkAxesActor::SPHERE_TIP ); }
    vtkGetMacro(mTipType, int);
    
    /** \brief Get the tip properties.    */
    vtkProperty *GetXAxisTipProperty();
    vtkProperty *GetYAxisTipProperty();
    vtkProperty *GetZAxisTipProperty();

    /** \brief Get the shaft properties.    */
    vtkProperty *GetXAxisShaftProperty();
    vtkProperty *GetYAxisShaftProperty();
    vtkProperty *GetZAxisShaftProperty();
    
  public:
    enum
    {
      CYLINDER_SHAFT,
      LINE_SHAFT,
    };
    enum
    {
      CONE_TIP,
      SPHERE_TIP,
    };

  protected:
    void               UpdateProps();

  protected:
    vtkSmartPointer<vtkCylinderSource>  mpCylinderSource;
    vtkSmartPointer<vtkLineSource>      mpLineSource;
    vtkSmartPointer<vtkConeSource>      mpConeSource;
    vtkSmartPointer<vtkSphereSource>    mpSphereSource;

    vtkSmartPointer<vtkActor>           mpXAxisShaft;
    vtkSmartPointer<vtkActor>           mpYAxisShaft;
    vtkSmartPointer<vtkActor>           mpZAxisShaft;

    vtkSmartPointer<vtkActor>           mpXAxisTip;
    vtkSmartPointer<vtkActor>           mpYAxisTip;
    vtkSmartPointer<vtkActor>           mpZAxisTip;
    
    double             mTotalLength[3];
    double             mNormalizedShaftLength[3];
    double             mNormalizedTipLength[3];
    double             mNormalizedLabelPosition[3];

    int                mShaftType;
    int                mTipType;    

    int                mConeResolution;
    int                mSphereResolution;
    int                mCylinderResolution;
    double             mConeRadius;
    double             mSphereRadius;
    double             mCylinderRadius;
};
