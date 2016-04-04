#pragma once

#include <envire_collision/ODECollision.hpp>
#include <envire/maps/MLSGrid.hpp>

#include <iostream>
#include <ode/collision.h>
#include <ode/common.h>
#include <ode/rotation.h>
#include <ode/matrix.h>
#include <ode/odemath.h>
#include <envire_collision/collision_kernel.h>
#include <envire_collision/collision_std.h>
#include <envire_collision/collision_util.h>
#include <envire_collision/config.h>

namespace envire { namespace collision
{

class MLSCollision : public ODECollision<envire::MLSGrid>
{
    ENVIRE_ODE_COLLISION_HEADER(MLSCollision)

protected:

    void getAABB (dGeomID o, dReal aabb[6], const boost::shared_ptr<envire::MLSGrid>& mls);

    int collide (dGeomID o1, dGeomID o2, int flags, dContactGeom *contact, 
						int skip, const boost::shared_ptr<envire::MLSGrid>& mls, int o2_class_id);
	int dCollideMlsfieldZone( const boost::shared_ptr<envire::MLSGrid>& mls, 
										   const int minX, const int maxX, const int minZ, const int maxZ, 
                                           dxGeom* o2, const int numMaxContactsPossible,
                                           int flags, dContactGeom* contact, 
                                           int skip );
                                           
    void  allocateHeightBuffer(size_t numX, size_t numZ);
						
				

};

}}
