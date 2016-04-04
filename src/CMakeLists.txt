find_package(Boost COMPONENTS system)

rock_library(envire_collider_mls
    SOURCES MLSCollision.cpp
    HEADERS MLSCollision.hpp
    DEPS_CMAKE Boost
    DEPS_PKGCONFIG ode envire_core envire envire_collision)
