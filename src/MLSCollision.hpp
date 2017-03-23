//
// Copyright (c) 2015, Deutsches Forschungszentrum für Künstliche Intelligenz GmbH.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//

#pragma once

#include <envire_collision/ODECollision.hpp>
#include <maps/grid/MLSMap.hpp>

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

class MLSCollision : public ODECollision<maps::grid::MLSMapKalman>
{
    ENVIRE_ODE_COLLISION_HEADER(MLSCollision)

public:    
    dReal       MinHeight;
    dReal       MaxHeight;    
    dReal       widthX;
    dReal       widthY;  
    dReal  		HalfWidthX;
    dReal  		HalfWidthY;  

      
protected:

    void getAABB (dGeomID o, dReal aabb[6], const boost::shared_ptr<maps::grid::MLSMapKalman>& mls);    

    int collide (dGeomID o1, dGeomID o2, int flags, dContactGeom *contact, 
						int skip, const boost::shared_ptr<maps::grid::MLSMapKalman>& mls, int o2_class_id);
	int dCollideSphereMls( const boost::shared_ptr<maps::grid::MLSMapKalman>& mls, 
										   const int minX, const int maxX, const int minZ, const int maxZ, 
                                           dxGeom* o2, int flags, dContactGeom* contact, int skip );
    dContactGeom   mls_contacts;	
    
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
