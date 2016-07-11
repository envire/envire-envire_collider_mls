#find_package(Boost COMPONENTS system)

#rock_testsuite(test_suite suite.cpp test_mlscollider.cpp
				  #DEPS_CMAKE Boost
                #DEPS envire_collider_mls)
			
#rock_executable(test2 test2_mlscollider.cpp
    #DEPS_CMAKE Boost
    #DEPS envire_collider_mls
    #DEPS_PKGCONFIG maps)			

rock_testsuite(test3 suite.cpp test3_mlscollider.cpp                
			DEPS envire_collider_mls			
			DEPS_PKGCONFIG maps)
add_subdirectory(mlsdata)
file(COPY mlsdata DESTINATION ./)

#rock_testsuite(test4 suite.cpp test4_mlscollider.cpp                
			#DEPS envire_collider_mls
			#DEPS_PKGCONFIG maps)
