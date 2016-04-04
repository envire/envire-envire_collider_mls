#include "MLSCollision.hpp"

#define dMIN(A,B)  ((A)>(B) ? (B) : (A))
#define dMAX(A,B)  ((A)>(B) ? (A) : (B))

using namespace envire::collision;

/*
void MLSCollision::dCreateMLSData(const boost::shared_ptr< envire::MLSGrid >& mls)                                 
{
    dUASSERT( d, "argument not Mlsfield data" );
    dIASSERT( pCallback );
    dIASSERT( widthSamples >= 2 );	// Ensure we're making something with at least one cell.
    dIASSERT( depthSamples >= 2 );

    // callback
    d->m_nGetHeightMode = 0;
    d->m_pUserData = pUserData;
    d->m_pGetHeightCallback = pCallback;
    dIASSERT( d->m_pUserData == pUserData );    
    
    (*pCallback)(pUserData, 1, 1);
    (*d->m_pGetHeightCallback)(d->m_pUserData, 2, 2);    

    // set info
    SetData( widthSamples, depthSamples, width, depth, scale, offset, thickness, bWrap );

    // default bounds
    d->m_fMinHeight = -dInfinity;
    d->m_fMaxHeight = dInfinity;
}
void MLSCollision::SetData( int nWidthSamples, int nDepthSamples,
                                dReal fWidth, dReal fDepth,
                                dReal fScale, dReal fOffset, dReal fThickness,
                                int bWrapMode )
{
    dIASSERT( fWidth > REAL( 0.0 ) );
    dIASSERT( fDepth > REAL( 0.0 ) );
    dIASSERT( nWidthSamples > 0 );
    dIASSERT( nDepthSamples > 0 );

    // x,z bounds
    m_fWidth = fWidth;
    m_fDepth = fDepth;

    // cache half x,z bounds
    m_fHalfWidth = fWidth / REAL( 2.0 );
    m_fHalfDepth = fDepth / REAL( 2.0 );

    // scale and offset
    m_fScale = fScale;
    m_fOffset = fOffset;

    // infinite min height bounds
    m_fThickness = fThickness;

    // number of vertices per side
    m_nWidthSamples = nWidthSamples;
    m_nDepthSamples = nDepthSamples;

    m_fSampleWidth = m_fWidth / ( m_nWidthSamples - REAL( 1.0 ) );
    m_fSampleDepth = m_fDepth / ( m_nDepthSamples - REAL( 1.0 ) );

    m_fSampleZXAspect = m_fSampleDepth / m_fSampleWidth;

    m_fInvSampleWidth = REAL( 1.0 ) / m_fSampleWidth;
    m_fInvSampleDepth = REAL( 1.0 ) / m_fSampleDepth;

    // finite or repeated terrain?
    m_bWrapMode = bWrapMode;
}
*/

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
	/*
	//size_t cellSizeX, cellSizeY;
	//double scalex, scaley;	
	//REAL( 1.0 ) / m_fSampleWidth;
    unsigned long size;
    int x, y;
    //int height, width;
    height =140;
    width =140;       
    //dReal targetWidth,targetHeight, scale;
    targetWidth = 14.0;
    targetHeight = 14.0;
    scale = 1.0;
    */
			o2->computeAABB();
     // To narrow scope of following variables
        const dReal fInvSampleWidth = REAL(1.0) / mls->getScaleX();//terrain->m_p_data->m_fInvSampleWidth;
        int nMinX = (int)dFloor(dNextAfter(o2->aabb[0] * fInvSampleWidth, -dInfinity));
     std::cout << "\n nMinX, o2->aabb[0], fInvSampleWidth, -dInfinity : " << nMinX <<","<<
      o2->aabb[0] <<","<< fInvSampleWidth<<","<< -dInfinity<<std::endl;       
        int nMaxX = (int)dCeil(dNextAfter(o2->aabb[1] * fInvSampleWidth, dInfinity));
     std::cout << "\n getCellSizeX, type nMaxX, o2->aabb[1], fInvSampleWidth, dInfinity : " << mls->getScaleX() <<"," <<nMaxX <<","<<
      o2->aabb[1] <<","<< fInvSampleWidth<<","<< dInfinity<<std::endl;    
              
        const dReal fInvSampleDepth = REAL( 1.0 ) / mls->getScaleY();//terrain->m_p_data->m_fInvSampleDepth;
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
        //numTerrainContacts += dCollideMlsfieldZone(
            //nMinX,nMaxX,nMinZ,nMaxZ,o2,numMaxTerrainContacts - numTerrainContacts,
            //flags,CONTACT(contact,numTerrainContacts*skip),skip	);
      
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
