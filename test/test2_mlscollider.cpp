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
    printf("at..%lf resol..(%lf %lf)\n", test, mls_kalman.getResolution().x(),mls_kalman.getResolution().y() );

    printf("index (%d %d)\n", mls_kalman.getNumCells().x(),mls_kalman.getNumCells().y() );	    

//	maps::grid::StandaloneVisualizer viz;

//	viz.updateData(mls_kalman);

	//while(viz.wait(1000))
	//{
		//// waiting
	//}

}

