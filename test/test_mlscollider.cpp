#include <boost/test/unit_test.hpp>
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
	
		//printf("mls.cellSizeX = (%d %d)\n",	mls->getCellSizeX(),	mls->getCellSizeY());

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
        dxSphere* geom_sphere = new dxSphere(0, 0.1);
		dGeomSetPosition (geom_sphere,0,0,0);        
        
        for(int i=1;i<100;i++)
        {

        //dGeomSetPosition (geom_sphere,0,0,0-0.1*(float)i);
                
        dContactGeom cg[10];
        int points = dCollide(geom_mls, geom_sphere, 1,  &cg[0], sizeof cg[0]);  
        printf("(NumOfCollision %d) (%f %f %f)\n",points,cg[0].pos[0],cg[0].pos[1],cg[0].pos[2]);
		}        
        dGeomDestroy(geom_mls);
        dGeomDestroy(geom_sphere);

    }
    dCloseODE();
}
