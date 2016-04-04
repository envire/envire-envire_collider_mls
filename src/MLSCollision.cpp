#include "MLSCollision.hpp"

#define dMIN(A,B)  ((A)>(B) ? (B) : (A))
#define dMAX(A,B)  ((A)>(B) ? (A) : (B))

using namespace envire::collision;

int MLSCollision::dCollideMlsfieldZone( const boost::shared_ptr<envire::MLSGrid>& mls,
										   const int minX, const int maxX, const int minZ, const int maxZ, 
                                           dxGeom* o2, const int numMaxContactsPossible,
                                           int flags, dContactGeom* contact, 
                                           int skip )
{

    dContactGeom *pContact = 0;
    int  x, z;
    // check if not above or inside terrain first
    // while filling a Mlsmap partial temporary buffer
    const unsigned int numX = (maxX - minX) + 1;
    const unsigned int numZ = (maxZ - minZ) + 1;
    const dReal minO2Height = o2->aabb[2];
    const dReal maxO2Height = o2->aabb[3];
    unsigned int x_local, z_local;
    dReal maxY = - dInfinity;
    dReal minY = dInfinity;
    
    std::cout << "numX numZ = " << numX << "," << numZ <<std::endl;
    
//return 0;	
}

int MLSCollision::collide(dGeomID o1, dGeomID o2, int flags, dContactGeom* contact, int skip, const boost::shared_ptr< envire::MLSGrid >& mls, int o2_class_id)
{		
		o2->computeAABB();
     // To narrow scope of following variables
        const dReal fInvSampleWidth = REAL(1.0) / mls->getScaleX();
        int nMinX = (int)dFloor(dNextAfter(o2->aabb[0] * fInvSampleWidth, -dInfinity));
        int nMaxX = (int)dCeil(dNextAfter(o2->aabb[1] * fInvSampleWidth, dInfinity));
        const dReal fInvSampleDepth = REAL( 1.0 ) / mls->getScaleY();
        int nMinZ = (int)dFloor(dNextAfter(o2->aabb[4] * fInvSampleDepth, -dInfinity));
        int nMaxZ = (int)dCeil(dNextAfter(o2->aabb[5] * fInvSampleDepth, dInfinity));

        //if ( !wrapped )
        //{
            nMinX = dMAX( nMinX, 0 );
            nMaxX = dMIN( nMaxX, mls->getCellSizeX() - 1);  //select overlabing area between o1 and o2
            nMinZ = dMAX( nMinZ, 0 );
            nMaxZ = dMIN( nMaxZ, mls->getCellSizeY() - 1);
     std::cout << "\n ...REAL 0.1 nMinX, nMaxX, nMinZ, nMaxZ : " << nMinX <<","<< nMaxX <<","<< nMinZ <<","<< nMaxZ<<std::endl;    
      
            dIASSERT ((nMinX < nMaxX) && (nMinZ < nMaxZ));
        //}
		int numTerrainContacts = 0;
		int numTerrainOrigContacts = 0;
		int numMaxTerrainContacts = 30;    
        
        numTerrainOrigContacts = numTerrainContacts;
        numTerrainContacts += dCollideMlsfieldZone(mls,
            nMinX,nMaxX,nMinZ,nMaxZ,o2,numMaxTerrainContacts - numTerrainContacts,
            flags,CONTACT(contact,numTerrainContacts*skip),skip	);
      
        dIASSERT( numTerrainContacts <= numMaxTerrainContacts );
    
    return 0;
}

void MLSCollision::getAABB(dGeomID o, dReal aabb[6], const boost::shared_ptr< envire::MLSGrid >& _user_data)
{
    //Eigen::Matrix<dReal, 3, 1> min = _user_data->min();
    //Eigen::Matrix<dReal, 3, 1> max = _user_data->max();
    //aabb[0] = min[0];
    //aabb[1] = max[0];
    //aabb[2] = min[1];
    //aabb[3] = max[1];
    //aabb[4] = min[2];
    //aabb[5] = max[2];
}
