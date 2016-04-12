#include <boost/test/unit_test.hpp>
#include <iostream>
#include <ode/collision.h>
#include <ode/common.h>
//#include <ode/rotation.h>
//#include <ode/matrix.h>
#include <ode/odemath.h>
#include <envire_collision/collision_kernel.h>
#include <envire_collision/collision_std.h>
#include <envire_collision/collision_util.h>
#include <envire_collision/config.h>

#include <envire_core/items/Item.hpp>
#include <envire_collision/Exceptions.hpp>
#include <envire_collider_mls/MLSCollision.hpp>

#include <envire/Core.hpp>
#include <envire/maps/GridBase.hpp>
#include <envire/maps/MLSGrid.hpp>
#include <envire/operators/MLSToGrid.hpp>
#include <envire/operators/MLSToMLSGeom.hpp>
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

        // check interface
        BOOST_CHECK(c->getGeomID() >= dFirstUserClass);
        BOOST_CHECK(dGeomGetClass(geom_mls) == c->getGeomID());

        boost::shared_ptr<envire::MLSGrid> user_data_a2 = c->getUserData(geom_mls);

        //BOOST_CHECK(user_data_a.use_count() == 3);
        BOOST_CHECK(mls == user_data_a2);

        // create second geom
        dxSphere* geom_sphere = new dxSphere(0, 0.5);
		float ini_z = 1.0f;
        int maxNumContacts = 5;
		dContact contact[maxNumContacts];   // the number of maximum contacts per a collision  
		
    dReal pos[3]={0.0f, 0.0f, 0.0f};  

        for(int k=0;k<100;k++)
        {
			dGeomSetPosition (geom_sphere,0.1,0.1,ini_z-0.1*(float)k);
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
