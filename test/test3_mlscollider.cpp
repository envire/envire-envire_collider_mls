
#include <fstream>
#include <boost/archive/polymorphic_binary_iarchive.hpp>
#include <maps/grid/MLSMap.hpp>

#include <boost/test/unit_test.hpp>
#include <envire_collider_mls/MLSCollision.hpp>

#include <boost/shared_ptr.hpp>

using namespace envire::collision;

		
BOOST_AUTO_TEST_CASE(test_box_collision)
{

    dInitODE();
    {
		MLSCollision* c = MLSCollision::getInstance();
		BOOST_CHECK(c != NULL);
		BOOST_CHECK(c->getGeomID() == -1);
	
		std::ifstream input("MLSMapKalman_waves.bin",  std::ios::binary);
		boost::archive::polymorphic_binary_iarchive  ia(input);
	
		maps::grid::MLSMapKalman mls_kalman;
		ia >> mls_kalman;
	
	    boost::shared_ptr<maps::grid::MLSMapKalman> mls(&mls_kalman);     
	
 	    BOOST_TEST_MESSAGE("mls resolution: " << mls->getResolution().x() <<", "<< mls->getResolution().y());   
        BOOST_TEST_MESSAGE("mls cell number: "<< mls->getNumCells().x() <<", "<< mls->getNumCells().y());
 	    BOOST_TEST_MESSAGE("mls size: "<< mls->getSize().x()<<", "<<mls->getSize().y());   
        BOOST_TEST_MESSAGE("mls mean value on (x,y): "<< mls->at(0,0).begin()->mean);
    	            
		// create first geom
		dGeomID geom_mls = c->createNewCollisionObject(mls);
		BOOST_CHECK(geom_mls->type == 14);
	
        c->widthX  = mls->getSize().x();
        c->widthY  = mls->getSize().y();   		
      
        geom_mls->aabb[0] = -10.0;					geom_mls->aabb[1] = +10.0;
        geom_mls->aabb[2] = -10.0;					geom_mls->aabb[3] = +10.0;                   
        geom_mls->aabb[4] = -2.0;					geom_mls->aabb[5] = 2.0;
        
        // check interface
        BOOST_CHECK(c->getGeomID() >= dFirstUserClass);
        BOOST_CHECK(dGeomGetClass(geom_mls) == c->getGeomID());

        boost::shared_ptr<maps::grid::MLSMapKalman> user_data_a2 = c->getUserData(geom_mls);

        //BOOST_CHECK(user_data_a.use_count() == 3);
        BOOST_CHECK(mls == user_data_a2);

        // create second geom
        dxSphere* geom_sphere = new dxSphere(0, 0.05);
		float ini_z = 1.0f;
        int maxNumContacts = 5;
		dContact contact[maxNumContacts];   // the number of maximum contacts per a collision  
		

        for(int k=0;k<100;k++)
        {
			dGeomSetPosition (geom_sphere,0.1,0.0,ini_z-0.01*(float)k);
			const dReal *pos2 = dGeomGetPosition(geom_sphere);
			BOOST_TEST_MESSAGE("pos of geom_sphere: "<< pos2[2]);  			
			
			if(int numc = dCollide(geom_mls, geom_sphere, maxNumContacts,  &contact[0].geom, sizeof(dContact)))
			{ 
              for (int i=0; i<numc; i++) {
			   BOOST_TEST_MESSAGE("collision x: "<< contact[i].geom.pos[0]<<", y: "<<contact[i].geom.pos[1]<<", z: "<<contact[i].geom.pos[2]); 			   
			  }       
			}

		}        
        dGeomDestroy(geom_mls);
        dGeomDestroy(geom_sphere);
    }
    dCloseODE();

}
