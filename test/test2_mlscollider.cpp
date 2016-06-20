#include <fstream>
#include <boost/archive/polymorphic_binary_iarchive.hpp>
#include <maps/grid/MLSMap.hpp>


int main(int argc, char **argv) {
	if(argc < 2)
	{
		std::cerr << "Usage " << argv[0] << " filename\n";
		exit(1);
	}


	std::ifstream input(argv[1],  std::ios::binary);
	boost::archive::polymorphic_binary_iarchive  ia(input);

	maps::grid::MLSMapKalman mls_kalman;

	ia >> mls_kalman;
	
    mls_kalman.getLocalFrame().translation() << 0.5*mls_kalman.getSize(), 0;	
    double test = mls_kalman.at(0,0).begin()->mean;
    printf("at..%lf\n", test);

//	maps::grid::StandaloneVisualizer viz;

//	viz.updateData(mls_kalman);

	//while(viz.wait(1000))
	//{
		//// waiting
	//}

}


//BOOST_AUTO_TEST_CASE(test_box_collision)
//{


/*		
        MLSCollision* c = MLSCollision::getInstance();
        BOOST_CHECK(c != NULL);
        BOOST_CHECK(c->getGeomID() == -1);
*/        
		//std::string env_path("mls_data");
		//std::string mls_map_id("/mls-grid");
	
			
		//boost::scoped_ptr<envire::Environment> env(envire::Environment::unserialize(env_path));   
		//envire::MLSGrid::Ptr mlsgrid_ptr(env->getItem<envire::MLSGrid>(mls_map_id));		
		//boost::shared_ptr<envire::MLSGrid> mls(mlsgrid_ptr.get(), NullDeleter());
	
//		std::string env_path("MLSMapKalman_waves.bin");	
//	std::ifstream input(argv[1],  std::ios::binary);
//	boost::archive::polymorphic_binary_iarchive  ia(env_path);
//	maps::grid::MLSMapSloped mls;


	//maps::grid::MLSMapKalman mls;

	//ia >> mls;
	
    //mls.getLocalFrame().translation() << 0.5*mls.getSize(), 0;	
    //double test = mls.at(0,0).begin()->mean;
    //printf("at..%lf\n", test);	
	
	/*
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
*/

//}
