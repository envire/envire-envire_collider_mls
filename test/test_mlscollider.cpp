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

#include <boost/test/unit_test.hpp>
#include <iostream>
#include <ode/collision.h>
#include <ode/common.h>
//#include <ode/rotation.h>
//#include <ode/matrix.h>
#include <ode/odemath.h>
#include <envire_collider_mls/collision_kernel.h>
#include <envire_collider_mls/collision_std.h>
#include <envire_collider_mls/collision_util.h>
#include <envire_collider_mls/config.h>

#include <envire_core/items/Item.hpp>
#include <envire_collision/Exceptions.hpp>
#include <envire_collider_mls/MLSCollision.hpp>

#include <envire/Core.hpp>
#include <envire/maps/GridBase.hpp>
#include <envire/maps/MLSGrid.hpp>
#include <envire/operators/MLSToGrid.hpp>

#include <boost/scoped_ptr.hpp>
#include <boost/intrusive_ptr.hpp>



using namespace envire::collision;
using namespace envire;

struct NullDeleter {template<typename T> void operator()(T*) {}};

BOOST_AUTO_TEST_CASE(test_box_collision)
{

    dInitODE();
    {
		
        MLSCollision* c = MLSCollision::getInstance();
        BOOST_CHECK(c != NULL);
        BOOST_CHECK(c->getGeomID() == -1);
        
		std::string env_path("mls_data");
		std::string mls_map_id("/mls-grid");
	
			
		boost::scoped_ptr<envire::Environment> env(envire::Environment::unserialize(env_path));   
		envire::MLSGrid::Ptr mlsgrid_ptr(env->getItem<envire::MLSGrid>(mls_map_id));		
		boost::shared_ptr<envire::MLSGrid> mls(mlsgrid_ptr.get(), NullDeleter());
	
		printf("mls.cellSizeX = (%d %d)\n",	mls->getCellSizeX(),	mls->getCellSizeY());
	
        // create first geom
        dGeomID geom_mls = c->createNewCollisionObject(mls);
        //c->setTransformation(geom_a, Eigen::Affine3d::Identity());
        BOOST_CHECK(geom_mls->type == 14);
      
        c->widthX  = mls->getCellSizeX()*mls->getScaleX();
        c->widthY  = mls->getCellSizeY()*mls->getScaleY();   		
      
        geom_mls->aabb[0] = -10.0;					geom_mls->aabb[1] = +10.0;
        geom_mls->aabb[2] = -10.0;					geom_mls->aabb[3] = +10.0;                   
        geom_mls->aabb[4] = -2.0;					geom_mls->aabb[5] = 2.0;
        
        // check interface
        BOOST_CHECK(c->getGeomID() >= dFirstUserClass);
        BOOST_CHECK(dGeomGetClass(geom_mls) == c->getGeomID());

        boost::shared_ptr<envire::MLSGrid> user_data_a2 = c->getUserData(geom_mls);

        //BOOST_CHECK(user_data_a.use_count() == 3);
        BOOST_CHECK(mls == user_data_a2);

        // create second geom
        dxSphere* geom_sphere = new dxSphere(0, 0.05);
		float ini_z = 1.0f;
        int maxNumContacts = 5;
		dContact contact[maxNumContacts];   // the number of maximum contacts per a collision  
		

        for(int k=0;k<100;k++)
        {
			dGeomSetPosition (geom_sphere,0.0,0.0,ini_z-0.1*(float)k);
			const dReal *pos2 = dGeomGetPosition(geom_sphere);
			if(int numc = dCollide(geom_mls, geom_sphere, maxNumContacts,  &contact[0].geom, sizeof(dContact)))
			{ 
              for (int i=0; i<numc; i++) {
		       printf("(k z pos[2]: %d %f %f)(coll_num = %d) (%f %f %f)\n"
		       ,k,ini_z-0.1*(float)k,pos2[2],i,contact[i].geom.pos[0],contact[i].geom.pos[1],contact[i].geom.pos[2]);  
			  }       
			}

		}        
        dGeomDestroy(geom_mls);
        dGeomDestroy(geom_sphere);

    }
    dCloseODE();
}
