# envire-envire_collider_mls
This is a collision library of MLS (Multi-Level Surface) for a stand alone collision detection in Rock as well as for MARS simulator. Currently the MLS collider is able to be used for collision detection between box-shape MLS and sphere-shape object. Since MLS collider is basically an extension from the ODE collision, it has the following structures:
- Interface to ODE collision library via the user defined class (it is implemented in envire_collision library)
- SpaceCollider (Bounding Box: getAABB() )
- class-specific collision function (collide())

Example of using the MLS collider (without MARS):
envire-envire_collider_mls/test/test3_mlscollider.cpp

1. create mls collider instance and interface it to ODE
2. load mls data to mls class with shared_ptr
3. create mls geom and sphere geom
4. call class-specific collision function and get contact information

Example of using the MLS collider as a MARS plugin : 
simulation-envire_mars/plugins/test_mls/
simulation-envire_mars/plugins/envire_joints/

1. create mls collider instance and interface it to ODE

2. load mls data to mls class with shared_ptr
3. create mls geom and sphere geom
4. get space instance and add the mls geom to the space in MARS

5. create and set additional geom data for MARS
