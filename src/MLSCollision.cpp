#include "MLSCollision.hpp"

#define dMIN(A,B)  ((A)>(B) ? (B) : (A))
#define dMAX(A,B)  ((A)>(B) ? (A) : (B))

#define dOPESIGN(a, op1, op2,b) \
     (a)[0] op1 op2 ((b)[0]); \
     (a)[1] op1 op2 ((b)[1]); \
     (a)[2] op1 op2 ((b)[2]);

using namespace envire::collision;

int MLSCollision::dCollideMlsfieldZone( const boost::shared_ptr<envire::MLSGrid>& mls,
										   const int minX, const int maxX, const int minZ, const int maxZ, 
                                           dxGeom* o2, const int numMaxContactsPossible,
                                           int flags, dContactGeom* contact, 
                                           int skip )
{	

    dContactGeom *pContact = 0;
    int numTerrainContacts = 0;  
    unsigned int numCollidingRects = 0;      
    int  x, z;
    // check if not above or inside terrain first
    // while filling a Mlsmap partial temporary buffer
    const unsigned int numX = maxX - minX;
    const unsigned int numZ = maxZ - minZ;
    const dReal minO2Height = o2->aabb[2];
    const dReal maxO2Height = o2->aabb[3];
    unsigned int x_local, z_local;
    dReal maxY = - dInfinity;
    dReal minY = dInfinity;
    const unsigned int numRectMax = (maxX - minX) * (maxZ - minZ);  
    

    size_t alignedNumRect = AlignBufferSize(numRectMax, TEMP_RECTANGULAR_BUFFER_ELEMENT_COUNT_ALIGNMENT);
    MlsFieldRectangular *rects = new MlsFieldRectangular[alignedNumRect]; 
    MlsFieldRectangular rect;

  	bool isACollide = false;
  	bool isBCollide = false;
  	bool isCCollide = false;
  	bool isDCollide = false; 
	
	for ( x = minX, x_local = 0; x_local < numX; x++, x_local++)    
    {
		for ( z = minZ, z_local = 0; z_local < numZ; z++, z_local++) 	
		{
			int count =0;
		    for( MLSGrid::iterator cit = mls->beginCell(x,z); cit != mls->endCell(); cit++ )
		    {
				MLSGrid::SurfacePatch p( *cit );  
	            isACollide = p.mean > minO2Height;    			
					rect.vertices[0].vertex[0] = x * mls->getScaleX();
					rect.vertices[0].vertex[1] = p.mean;
					rect.vertices[0].vertex[2] = z * mls->getScaleY();	
printf("(x,z)=(%d %d) pos(%f %f %f)\n",x,z,rect.vertices[0].vertex[0],rect.vertices[0].vertex[1],rect.vertices[0].vertex[2]);							
		
	        }    
	 	    for( MLSGrid::iterator cit = mls->beginCell(x+1,z); cit != mls->endCell(); cit++ )
		    {
				MLSGrid::SurfacePatch p( *cit );  
	            isBCollide = p.mean > minO2Height; 
					rect.vertices[1].vertex[0] = (x+1) * mls->getScaleX();
					rect.vertices[1].vertex[1] = p.mean;
					rect.vertices[1].vertex[2] = z * mls->getScaleY();		          
printf("(x,z)=(%d %d) pos(%f %f %f)\n",x+1,z,rect.vertices[1].vertex[0],rect.vertices[1].vertex[1],rect.vertices[1].vertex[2]);					  
					
	        }    
		    for( MLSGrid::iterator cit = mls->beginCell(x,z+1); cit != mls->endCell(); cit++ )
		    {
				MLSGrid::SurfacePatch p( *cit );  
	            isCCollide = p.mean > minO2Height;  
					rect.vertices[2].vertex[0] = x * mls->getScaleX();
					rect.vertices[2].vertex[1] = p.mean;
					rect.vertices[2].vertex[2] = (z+1) * mls->getScaleY();	
printf("(x,z)=(%d %d) pos(%f %f %f)\n",x,z+1,rect.vertices[2].vertex[0],rect.vertices[2].vertex[1],rect.vertices[2].vertex[2]);									
	        } 
	 	    for( MLSGrid::iterator cit = mls->beginCell(x+1,z+1); cit != mls->endCell(); cit++ )
		    {
				MLSGrid::SurfacePatch p( *cit );  
	            isDCollide = p.mean > minO2Height;
					rect.vertices[3].vertex[0] = (x+1) * mls->getScaleX();
					rect.vertices[3].vertex[1] = p.mean;
					rect.vertices[3].vertex[2] = (z+1) * mls->getScaleY();		
printf("(x,z)=(%d %d) pos(%f %f %f)\n",x+1,z+1,rect.vertices[3].vertex[0],rect.vertices[3].vertex[1],rect.vertices[3].vertex[2]);								
	        }     
			if (isACollide || isBCollide || isCCollide || isDCollide)
			{  
				//printf("numCollidingCells = %d alignedNumRect = %d numRectMax = %d\n",numCollidingCells,alignedNumRect,numRectMax);
				rects[numCollidingRects++] = rect;	
//printf("......numCollidingCells = %d........\n",numCollidingRects-1);			
			}               

		}
	}   

	    int maxBoxNum = 4;
	    double boxlength = 0.5;
	    
		dxBox* colliding_box[maxBoxNum];
		for(int i=0; i<4; i++) colliding_box[i] = new dxBox (0,1,1,1);   //TODE space pointer has to be added
    
    for (unsigned int k = 0; k < numCollidingRects; k++)
    {
		for(unsigned int i=0;i<4;i++){   //we need to build 4 boxs per a rect (k*4 loops)
		   //set positions and size of Boxs A,B,C,D from collision
		   dVector3Copy(rects[k].vertices[i].vertex, colliding_box[i]->final_posr->pos); 
		   colliding_box[i]->side[0] = rects[k].vertices[1].vertex[0] - rects[k].vertices[0].vertex[0];
		   colliding_box[i]->side[1] = boxlength;	
		   colliding_box[i]->side[2] = rects[k].vertices[2].vertex[2] - rects[k].vertices[0].vertex[2];	
		   colliding_box[i]->final_posr->pos[0] -= (colliding_box[i]->side[0]/2);   
		   colliding_box[i]->final_posr->pos[1] = colliding_box[i]->final_posr->pos[1] - boxlength/2;  	
		   colliding_box[i]->final_posr->pos[2] -= (colliding_box[i]->side[2]/2); 	
										
		int collided = dCollideSphereBox (o2, colliding_box[i], flags, &mls_contacts, skip);			

		   if(collided && (numTerrainContacts < numMaxContactsPossible)) {	  			   
  		       pContact = CONTACT(contact, numTerrainContacts*skip);			   
		       dVector3Copy(mls_contacts.pos, pContact->pos);
               //create contact using Plane Normal
               dOPESIGN(pContact->normal, =, -, mls_contacts.normal);	                      
		       pContact->depth = mls_contacts.depth;	
       
 	           numTerrainContacts++;	
		   } 
		   
		}

	} 
	for(int i=0; i<maxBoxNum; i++) delete colliding_box[i];   
	delete[] rects;  
//	printf("dCollideMlsfieldZone.......numTerrainContacts = %d \n",numTerrainContacts);
	return numTerrainContacts;
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
     //std::cout << "\n ...REAL 0.1 nMinX, nMaxX, nMinZ, nMaxZ : " << nMinX <<","<< nMaxX <<","<< nMinZ <<","<< nMaxZ<<std::endl;    
       //std::cout << "\n ...Height =  " << mls->getHeight()<<std::endl;   
         
            dIASSERT ((nMinX < nMaxX) && (nMinZ < nMaxZ));
        //}
		int numTerrainContacts = 0;
		int numMaxTerrainContacts = flags;    
        
        numTerrainContacts += dCollideMlsfieldZone(mls,
            nMinX,nMaxX,nMinZ,nMaxZ,o2,numMaxTerrainContacts,
            flags,CONTACT(contact,numTerrainContacts*skip),skip	);
      
        dIASSERT( numTerrainContacts <= numMaxTerrainContacts );
    
    dContactGeom *pContact;

    for (int i = 0; i != numTerrainContacts; ++i )
    {
        pContact = CONTACT(contact,i*skip);
        pContact->g1 = o1;
        pContact->g2 = o2;
        pContact->side1 = -1; 
        pContact->side2 = -1;
    }
     return numTerrainContacts;   
}

void MLSCollision::getAABB(dGeomID o, dReal aabb[6], const boost::shared_ptr< envire::MLSGrid >& _user_data)
{
}

