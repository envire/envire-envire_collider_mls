#pragma once

#include <envire_collision/ODECollision.hpp>
#include <envire/maps/MLSGrid.hpp>

#include <iostream>
#include <ode/collision.h>
#include <ode/common.h>
#include <ode/rotation.h>
#include <ode/matrix.h>
#include <ode/odemath.h>
#include <envire_collider_mls/collision_kernel.h>
#include <envire_collider_mls/collision_std.h>
#include <envire_collider_mls/collision_util.h>
#include <envire_collider_mls/config.h>


#define TEMP_RECTANGULAR_BUFFER_ELEMENT_COUNT_ALIGNMENT 1

namespace envire { namespace collision
{

class MLSCollision : public ODECollision<envire::MLSGrid>
{
    ENVIRE_ODE_COLLISION_HEADER(MLSCollision)
   
protected:

    void getAABB (dGeomID o, dReal aabb[6], const boost::shared_ptr<envire::MLSGrid>& mls);    

    int collide (dGeomID o1, dGeomID o2, int flags, dContactGeom *contact, 
						int skip, const boost::shared_ptr<envire::MLSGrid>& mls, int o2_class_id);
	int dCollideSphereMls( const boost::shared_ptr<envire::MLSGrid>& mls, 
										   const int minX, const int maxX, const int minZ, const int maxZ, 
                                           dxGeom* o2, const int numMaxContactsPossible,
                                           int flags, dContactGeom* contact, 
                                           int skip );
    dContactGeom   mls_contacts;	
    dReal       MinHeight;
    dReal       MaxHeight;    
    dReal       widthX;
    dReal       widthY;  
    dReal  		HalfWidthX;
    dReal  		HalfWidthY;  
    
    int	WrapMode;           // Heightfield wrapping mode (0=finite, 1=infinite)

};

static inline size_t AlignBufferSize(size_t value, size_t alignment) { dIASSERT((alignment & (alignment - 1)) == 0); 
	return (value + (alignment - 1)) & ~(alignment - 1); }

class MlsFieldVertex
{
public:
    MlsFieldVertex(){};
    dVector3 vertex;
    bool is;    
};

class MlsFieldRectangular
{
public:
    MlsFieldRectangular(){};
    MlsFieldVertex   vertices[4];
};

}}
