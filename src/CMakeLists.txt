find_package(Boost COMPONENTS system)

rock_library(envire_collider_mls
    SOURCES MLSCollision.cpp
    HEADERS MLSCollision.hpp
            collision_kernel.h
            collision_std.h
            collision_util.h
            array.h
            config.h
            objects.h
            odeou.h
            odetls.h  
    DEPS_CMAKE Boost
    DEPS_PKGCONFIG maps ode envire_core envire_collision)
    
    
    
