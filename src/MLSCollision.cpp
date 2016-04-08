#include "MLSCollision.hpp"

#define dMIN(A,B)  ((A)>(B) ? (B) : (A))
#define dMAX(A,B)  ((A)>(B) ? (A) : (B))

#define dOPESIGN(a, op1, op2,b) \
     (a)[0] op1 op2 ((b)[0]); \
     (a)[1] op1 op2 ((b)[1]); \
     (a)[2] op1 op2 ((b)[2]);

using namespace envire::collision;

int MLSCollision::dCollideBoundingBox( const boost::shared_ptr<envire::MLSGrid>& mls,
										   const int minX, const int maxX, const int minY, const int maxY, 
                                           dxGeom* o2, const int numMaxContactsPossible,
                                           int flags, dContactGeom* contact, 
                                           int skip )
{	

    dContactGeom *pContact = 0;
    int numTerrainContacts = 0;  
    unsigned int numCollidingRects = 0;      
    int  x, y;
    // check if not above or inside terrain first
    // while filling a Mlsmap partial temporary buffer
    const unsigned int numX = maxX - minX;
    const unsigned int numY = maxY - minY;
    const dReal minO2Height = o2->aabb[4];
    const dReal maxO2Height = o2->aabb[5];
    unsigned int x_local, y_local;
    dReal maxZ = - dInfinity;
    dReal minZ = dInfinity;
    const unsigned int numRectMax = (maxX - minX) * (maxY - minY);  
    

    size_t alignedNumRect = AlignBufferSize(numRectMax, TEMP_RECTANGULAR_BUFFER_ELEMENT_COUNT_ALIGNMENT);
    MlsFieldRectangular *rects = new MlsFieldRectangular[alignedNumRect]; 
    MlsFieldRectangular rect;

  	bool isACollide = false;
  	bool isBCollide = false;
  	bool isCCollide = false;
  	bool isDCollide = false; 
	
	for ( x = minX, x_local = 0; x_local < numX; x++, x_local++)    
    {
		for ( y = minY, y_local = 0; y_local < numY; y++, y_local++) 	
		{
			int count =0;
		    for( MLSGrid::iterator cit = mls->beginCell(x,y); cit != mls->endCell(); cit++ )
		    {
				MLSGrid::SurfacePatch p( *cit );  
	            isACollide = p.mean > minO2Height;    			
					rect.vertices[0].vertex[0] = x * mls->getScaleX();
					rect.vertices[0].vertex[1] = y * mls->getScaleY();	 
					rect.vertices[0].vertex[2] = p.mean;
                maxZ = dMAX(maxZ, p.mean);
                minZ = dMIN(minZ, p.mean);            
printf("(x,y)=(%d %d) pos(%f %f %f)\n",x,y,rect.vertices[0].vertex[0],rect.vertices[0].vertex[1],rect.vertices[0].vertex[2]);							
		
	        }    
	 	    for( MLSGrid::iterator cit = mls->beginCell(x+1,y); cit != mls->endCell(); cit++ )
		    {
				MLSGrid::SurfacePatch p( *cit );  
	            isBCollide = p.mean > minO2Height; 
					rect.vertices[1].vertex[0] = (x+1) * mls->getScaleX();
					rect.vertices[1].vertex[1] = y * mls->getScaleY();
					rect.vertices[1].vertex[2] = p.mean;         
                maxZ = dMAX(maxZ, p.mean);
                minZ = dMIN(minZ, p.mean);  					
printf("(x,y)=(%d %d) pos(%f %f %f)\n",x+1,y,rect.vertices[1].vertex[0],rect.vertices[1].vertex[1],rect.vertices[1].vertex[2]);					  
					
	        }    
		    for( MLSGrid::iterator cit = mls->beginCell(x,y+1); cit != mls->endCell(); cit++ )
		    {
				MLSGrid::SurfacePatch p( *cit );  
	            isCCollide = p.mean > minO2Height;  
					rect.vertices[2].vertex[0] = x * mls->getScaleX();
					rect.vertices[2].vertex[1] = (y+1) * mls->getScaleY();
					rect.vertices[2].vertex[2] = p.mean; 
                maxZ = dMAX(maxZ, p.mean);
                minZ = dMIN(minZ, p.mean);  
printf("(x,y)=(%d %d) pos(%f %f %f)\n",x,y+1,rect.vertices[2].vertex[0],rect.vertices[2].vertex[1],rect.vertices[2].vertex[2]);									
	        } 
	 	    for( MLSGrid::iterator cit = mls->beginCell(x+1,y+1); cit != mls->endCell(); cit++ )
		    {
				MLSGrid::SurfacePatch p( *cit );  
	            isDCollide = p.mean > minO2Height;
					rect.vertices[3].vertex[0] = (x+1) * mls->getScaleX();
					rect.vertices[3].vertex[1] = (y+1) * mls->getScaleY();
					rect.vertices[3].vertex[2] = p.mean; 	
                maxZ = dMAX(maxZ, p.mean);
                minZ = dMIN(minZ, p.mean);  
printf("(x,y)=(%d %d) pos(%f %f %f)\n",x+1,y+1,rect.vertices[3].vertex[0],rect.vertices[3].vertex[1],rect.vertices[3].vertex[2]);								
	        }     
			if (isACollide || isBCollide || isCCollide || isDCollide)
			{  
				//printf("numCollidingCells = %d alignedNumRect = %d numRectMax = %d\n",numCollidingCells,alignedNumRect,numRectMax);
				rects[numCollidingRects++] = rect;	
//printf("......numCollidingCells = %d........\n",numCollidingRects-1);			
			}               

		}
	}   

        if (minO2Height - maxZ > -dEpsilon )    //TODO: make clear!!
        {
            //totally above Mlsfield
printf(".totally above Mlsfield...(minO2Height - maxZ > -dEpsilon) (%f %f %f)\n",minO2Height,maxZ,-dEpsilon);            
            return 0;
        }
        
        if (minZ - maxO2Height > -dEpsilon )
        {
            // totally under Mlsfield
            pContact = CONTACT(contact, 0);			//set pointer of dContactGeom....YH

            pContact->pos[0] = o2->final_posr->pos[0];
            pContact->pos[1] = o2->final_posr->pos[1];            
            pContact->pos[2] = minZ;


            pContact->normal[0] = 0;
            pContact->normal[1] = 0;
            pContact->normal[2] = -1;

            pContact->depth =  minZ - maxO2Height;

            pContact->side1 = -1;
            pContact->side2 = -1;
printf(".totally under Mlsfield...minZ - maxO2Height > -dEpsilon(%f %f %f)\n",minZ, minO2Height,-dEpsilon);
            return 1;
//			return numTerrainContacts++;
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
		   colliding_box[i]->side[1] = rects[k].vertices[2].vertex[1] - rects[k].vertices[0].vertex[1];			   
		   colliding_box[i]->side[2] = boxlength;	

		   colliding_box[i]->final_posr->pos[0] -= (colliding_box[i]->side[0]/2);   
		   colliding_box[i]->final_posr->pos[1] -= (colliding_box[i]->side[2]/2); 	
		   colliding_box[i]->final_posr->pos[2] = colliding_box[i]->final_posr->pos[2] - boxlength/2;  	
		   										
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
	printf("dCollideMlsfieldZone.......numTerrainContacts = %d \n",numTerrainContacts);
	return numTerrainContacts;
}

int MLSCollision::collide(dGeomID o1, dGeomID o2, int flags, dContactGeom* contact, int skip, const boost::shared_ptr< envire::MLSGrid >& mls, int o2_class_id)
{		
		o2->computeAABB();
        const dReal fInvScaleX = REAL(1.0) / mls->getScaleX();
        int nMinX = (int)dFloor(dNextAfter(o2->aabb[0] * fInvScaleX, -dInfinity));
        int nMaxX = (int)dCeil(dNextAfter(o2->aabb[1] * fInvScaleX, dInfinity));
        const dReal fInvScaleY = REAL( 1.0 ) / mls->getScaleY();
        int nMinY = (int)dFloor(dNextAfter(o2->aabb[4] * fInvScaleY, -dInfinity));
        int nMaxY = (int)dCeil(dNextAfter(o2->aabb[5] * fInvScaleY, dInfinity));

		nMinX = dMAX( nMinX, 0 );
		nMaxX = dMIN( nMaxX, mls->getCellSizeX() - 1);  //select overlabing area between o1 and o2
		nMinY = dMAX( nMinY, 0 );
		nMaxY = dMIN( nMaxY, mls->getCellSizeY() - 1);
	 
		dIASSERT ((nMinX < nMaxX) && (nMinY < nMaxY));

		int numTerrainContacts = 0;
		int numMaxTerrainContacts = flags;    
        
        numTerrainContacts += dCollideBoundingBox(mls,
            nMinX,nMaxX,nMinY,nMaxY,o2,numMaxTerrainContacts,
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

