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

int main(int argc, char **argv) {
	if(argc < 2)
	{
		std::cerr << "Usage " << argv[0] << " filename\n";
		exit(1);
	}


	std::ifstream input(argv[1],  std::ios::binary);
	boost::archive::binary_iarchive  ia(input);

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

