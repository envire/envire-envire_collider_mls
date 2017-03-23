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


#include <fstream>
#include <boost/archive/binary_iarchive.hpp>
#include <maps/grid/MLSMap.hpp>

#include <boost/test/unit_test.hpp>
#include <envire_collider_mls/MLSCollision.hpp>

#define	DEGTORAD			0.01745329251994329577f				//!< PI / 180.0, convert degrees to radians
//#include <boost/shared_ptr.hpp>

using namespace envire::collision;

		
BOOST_AUTO_TEST_CASE(test_box_collision)
{


    dInitODE();
    {
		MLSCollision* c = MLSCollision::getInstance();
		BOOST_CHECK(c != NULL);
		BOOST_CHECK(c->getGeomID() == -1);

		std::string env_path("./mlsdata/MLSMapKalman_waves.bin");
		std::ifstream input(env_path,  std::ios::binary);
		//std::ifstream input("MLSMapKalman_waves.bin",  std::ios::binary);		
		boost::archive::binary_iarchive  ia(input);
		
		maps::grid::MLSMapKalman mls_kalman;
		ia >> mls_kalman;
	
	    boost::shared_ptr<maps::grid::MLSMapKalman> mls(&mls_kalman);     
	
 	    BOOST_TEST_MESSAGE("mls resolution: " << mls->getResolution().x() <<", "<< mls->getResolution().y());   
        BOOST_TEST_MESSAGE("mls cell number: "<< mls->getNumCells().x() <<", "<< mls->getNumCells().y());
 	    BOOST_TEST_MESSAGE("mls size: "<< mls->getSize().x()<<", "<<mls->getSize().y());   
        BOOST_TEST_MESSAGE("mls mean value on (x,y): "<< mls->at(0,0).begin()->getMean());
	    	            
		// create first geom
		dGeomID geom_mls = c->createNewCollisionObject(mls);
		BOOST_CHECK(geom_mls->type == 14);
	
        c->widthX  = mls->getSize().x();
        c->widthY  = mls->getSize().y();   		
      
    dVector3 pos;
	pos[ 0 ] = 0;
	pos[ 1 ] = 0;
	pos[ 2 ] = 0;

	// Rotate so Z is up, not Y (which is the default orientation)
	dMatrix3 R;
	dRSetIdentity( R );
	//dRFromAxisAndAngle( R, 1, 0, 0, DEGTORAD * 90 );

	// Place it.
	dGeomSetRotation( geom_mls, R );
	dGeomSetPosition( geom_mls, pos[0], pos[1], pos[2] );
        
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
			dGeomSetPosition (geom_sphere,0.0,0.0,ini_z-0.01*(float)k);
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
