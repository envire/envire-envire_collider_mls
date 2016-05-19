#include "MLSCollision.hpp"

#define dMIN(A,B)  ((A)>(B) ? (B) : (A))
#define dMAX(A,B)  ((A)>(B) ? (A) : (B))

// Three-way MIN and MAX
#define dMIN3(A,B,C)	( (A)<(B) ? dMIN((A),(C)) : dMIN((B),(C)) )
#define dMAX3(A,B,C)	( (A)>(B) ? dMAX((A),(C)) : dMAX((B),(C)) )


#define dOPESIGN(a, op1, op2,b) \
     (a)[0] op1 op2 ((b)[0]); \
     (a)[1] op1 op2 ((b)[1]); \
     (a)[2] op1 op2 ((b)[2]);
     
//#define DHEIGHTFIELD_CORNER_ORIGIN
#define GEOM_PLACEABLE 1

using namespace envire::collision;

int MLSCollision::dCollideSphereMls( const boost::shared_ptr<envire::MLSGrid>& mls,
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
			MLSGrid::iterator A = mls->beginCell(x,y);
			if (A == mls->endCell()) {
						rect.vertices[0].vertex[0] = x * mls->getScaleX();
						rect.vertices[0].vertex[1] = y * mls->getScaleY();
						rect.vertices[0].is = false;	 
			}
			else{
				    for( A; A != mls->endCell(); A++ )
				    {
						MLSGrid::SurfacePatch p( *A );  
			            isACollide = p.mean > minO2Height;  
							rect.vertices[0].vertex[0] = x * mls->getScaleX();
							rect.vertices[0].vertex[1] = y * mls->getScaleY();	 
							rect.vertices[0].vertex[2] = p.mean;
							rect.vertices[0].is = true;	
								
		                maxZ = dMAX(maxZ, p.mean);
		                minZ = dMIN(minZ, p.mean);            
				
			        }   
		    } 
			MLSGrid::iterator B = mls->beginCell(x+1,y);
			if (B == mls->endCell()) {
						rect.vertices[1].vertex[0] = (x+1) * mls->getScaleX();
						rect.vertices[1].vertex[1] = y * mls->getScaleY();	 
						rect.vertices[1].is = false;						
			}
			else{
				    for( B; B != mls->endCell(); B++ )
				    {
						MLSGrid::SurfacePatch p( *B );  
			            isACollide = p.mean > minO2Height;  
							rect.vertices[1].vertex[0] = (x+1) * mls->getScaleX();
							rect.vertices[1].vertex[1] = y * mls->getScaleY();	 
							rect.vertices[1].vertex[2] = p.mean;
							rect.vertices[1].is = true;	
								
		                maxZ = dMAX(maxZ, p.mean);
		                minZ = dMIN(minZ, p.mean);            
				
			        }   
		    } 
			MLSGrid::iterator C = mls->beginCell(x,y+1);
			if (C == mls->endCell()) {
						rect.vertices[2].vertex[0] = x * mls->getScaleX();
						rect.vertices[2].vertex[1] = (y+1) * mls->getScaleY();	
						rect.vertices[2].is = false; 
			}
			else{
				    for( C; C != mls->endCell(); C++ )
				    {
						MLSGrid::SurfacePatch p( *C );  
			            isACollide = p.mean > minO2Height;  
							rect.vertices[2].vertex[0] = x * mls->getScaleX();
							rect.vertices[2].vertex[1] = (y+1) * mls->getScaleY();	 
							rect.vertices[2].vertex[2] = p.mean;
							rect.vertices[2].is = true;	
								
		                maxZ = dMAX(maxZ, p.mean);
		                minZ = dMIN(minZ, p.mean);            
				
			        }   
		    }
			MLSGrid::iterator D = mls->beginCell(x+1,y+1);
			if (D == mls->endCell()) {
						rect.vertices[3].vertex[0] = (x+1) * mls->getScaleX();
						rect.vertices[3].vertex[1] = (y+1) * mls->getScaleY();	
						rect.vertices[3].is = false; 
			}
			else{
				    for( D; D != mls->endCell(); D++ )
				    {
						MLSGrid::SurfacePatch p( *D );  
			            isACollide = p.mean > minO2Height;  
							rect.vertices[3].vertex[0] = (x+1) * mls->getScaleX();
							rect.vertices[3].vertex[1] = (y+1) * mls->getScaleY();	 
							rect.vertices[3].vertex[2] = p.mean;
							rect.vertices[3].is = true;	
								
		                maxZ = dMAX(maxZ, p.mean);
		                minZ = dMIN(minZ, p.mean);            
				
			        }   
		    } 		     		    		    

     
			if (isACollide || isBCollide || isCCollide || isDCollide)
			{  
				printf("numCollidingRects = %d alignedNumRect = %d numRectMax = %d\n",numCollidingRects,alignedNumRect,numRectMax);
				rects[numCollidingRects++] = rect;		
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
printf(".totally under Mlsfield...minZ - maxO2Height > -dEpsilon(%f %f %f)\n",minZ, maxO2Height,-dEpsilon);
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

			if(rect.vertices[i].is == false) rect.vertices[i].vertex[2] = minZ;  //in case that mls has no value
			
		   //set positions and size of Boxs A,B,C,D from collision
		   dVector3Copy(rects[k].vertices[i].vertex, colliding_box[i]->final_posr->pos); 
		   colliding_box[i]->side[0] = mls->getScaleX(); //abs(rects[k].vertices[1].vertex[0] - rects[k].vertices[0].vertex[0]);
		   colliding_box[i]->side[1] = mls->getScaleY();//abs(rects[k].vertices[2].vertex[1] - rects[k].vertices[0].vertex[1]);			   
		   colliding_box[i]->side[2] = rects[k].vertices[i].vertex[2]; //boxlength;	

		   colliding_box[i]->final_posr->pos[0] -= (colliding_box[i]->side[0]/2);   
		   colliding_box[i]->final_posr->pos[1] -= (colliding_box[i]->side[1]/2); 	
		   colliding_box[i]->final_posr->pos[2] /= 2; // boxlength/2;  	
		   										
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
	dIASSERT( skip >= (int)sizeof(dContactGeom) );
    dIASSERT( o1->type == 14 );
    dIASSERT((flags & NUMC_MASK) >= 1);
	      
    int numMaxTerrainContacts = (flags & NUMC_MASK);

    dVector3 posbak;
    dMatrix3 Rbak;
    dReal aabbbak[6];
    int gflagsbak;
    dVector3 pos0,pos1;
    dMatrix3 R1;

    int numTerrainContacts = 0;
    int numTerrainOrigContacts = 0;

    //@@ Should find a way to set reComputeAABB to false in default case
    // aka DHEIGHTFIELD_CORNER_ORIGIN not defined and terrain not PLACEABLE
    // so that we can free some memory and speed up things a bit
    // while saving some precision loss 
#ifndef DHEIGHTFIELD_CORNER_ORIGIN

    const bool reComputeAABB = true;
#else
    const bool reComputeAABB = ( terrain->gflags & GEOM_PLACEABLE ) ? true : false;
#endif //DHEIGHTFIELD_CORNER_ORIGIN

    if (reComputeAABB)
    {
        // Backup original o2 position, rotation and AABB.
        dVector3Copy( o2->final_posr->pos, posbak );
        dMatrix3Copy( o2->final_posr->R, Rbak );
        memcpy( aabbbak, o2->aabb, sizeof( dReal ) * 6 );
        gflagsbak = o2->gflags;
    }
	
    //if ( o1->gflags & GEOM_PLACEABLE )
    //{	
        // Transform o2 into heightfield space.
        dSubtractVectors3( pos0, o2->final_posr->pos, o1->final_posr->pos );
        dMultiply1_331( pos1, o1->final_posr->R, pos0 );
        dMultiply1_333( R1, o1->final_posr->R, o2->final_posr->R );

        // Update o2 with transformed position and rotation.
        dVector3Copy( pos1, o2->final_posr->pos );
        dMatrix3Copy( R1, o2->final_posr->R );
	//}

          
#ifndef DHEIGHTFIELD_CORNER_ORIGIN
    o2->final_posr->pos[ 0 ] += HalfWidthX;
    o2->final_posr->pos[ 1 ] += HalfWidthY;
#endif // DHEIGHTFIELD_CORNER_ORIGIN   
	
		
    // Rebuild AABB for O2
    if (reComputeAABB)
        o2->computeAABB();
		
    //check if inside boundaries
    // using O2 aabb
    //  aabb[6] is (minx, maxx, miny, maxy, minz, maxz) 
    const bool wrapped = WrapMode != 0;

    if ( !wrapped )
    {
        if (    o2->aabb[0] > widthX //MinX
            ||  o2->aabb[2] > widthY)//MinY>
            goto dCollideHeightfieldExit;

        if (    o2->aabb[1] < 0 //MaxX
            ||  o2->aabb[3] < 0)//MaxY
            goto dCollideHeightfieldExit;
    }		
		
	{
        const dReal fInvScaleX = REAL(1.0) / mls->getScaleX();
        int nMinX = (int)dFloor(dNextAfter(o2->aabb[0] * fInvScaleX, -dInfinity));
        int nMaxX = (int)dCeil(dNextAfter(o2->aabb[1] * fInvScaleX, dInfinity));
        
 //        int nMaxX = (int)dCeil(dNextAfter(o2->aabb[1] * fInvSampleWidth, dInfinity));       
        
        const dReal fInvScaleY = REAL( 1.0 ) / mls->getScaleY();
        int nMinY = (int)dFloor(dNextAfter(o2->aabb[2] * fInvScaleY, -dInfinity));
        int nMaxY = (int)dCeil(dNextAfter(o2->aabb[3] * fInvScaleY, dInfinity));   //TODO: test fInvScaleY size!!


		nMinX = dMAX( nMinX, 0 );
		nMaxX = dMIN( nMaxX, mls->getCellSizeX() - 1);  //select overlabing area between o1 and o2
		nMinY = dMAX( nMinY, 0 );
		nMaxY = dMIN( nMaxY, mls->getCellSizeY() - 1);
	
		
 
		dIASSERT ((nMinX < nMaxX) && (nMinY < nMaxY));

		int numMaxTerrainContacts = flags;    
 	       
        numTerrainContacts += dCollideSphereMls(mls,
            nMinX,nMaxX,nMinY,nMaxY,o2,numMaxTerrainContacts,
            flags,CONTACT(contact,numTerrainContacts*skip),skip	);
    printf("num collided---func---------- %d\n", numTerrainContacts);
        dIASSERT( numTerrainContacts <= numMaxTerrainContacts );
    }
        dContactGeom *pContact;

    for (int i = numTerrainOrigContacts; i != numTerrainContacts; ++i )
    {
        pContact = CONTACT(contact,i*skip);
        pContact->g1 = o1;
        pContact->g2 = o2;
        pContact->side1 = -1; 
        pContact->side2 = -1;
    }
    
dCollideHeightfieldExit:
    if (reComputeAABB)
    {
        // Restore o2 position, rotation and AABB
        dVector3Copy( posbak, o2->final_posr->pos );
        dMatrix3Copy( Rbak, o2->final_posr->R );
        memcpy( o2->aabb, aabbbak, sizeof(dReal)*6 );
        o2->gflags = gflagsbak;
        
        printf("dCollideHeightfieldExit:..\n");

        //
        // Transform Contacts to World Space
        //
        if ( o1->gflags & GEOM_PLACEABLE )
        {
            for ( int i = 0; i < numTerrainContacts; ++i )
            {
                pContact = CONTACT(contact,i*skip);
                dCopyVector3( pos0, pContact->pos );

#ifndef DHEIGHTFIELD_CORNER_ORIGIN
                pos0[ 0 ] -= HalfWidthX;
                pos0[ 1 ] -= HalfWidthY;
#endif // !DHEIGHTFIELD_CORNER_ORIGIN

                dMultiply0_331( pContact->pos, o1->final_posr->R, pos0 );

                dAddVectors3( pContact->pos, pContact->pos, o1->final_posr->pos );
                dCopyVector3( pos0, pContact->normal );

                dMultiply0_331( pContact->normal, o1->final_posr->R, pos0 );
            }
        }
#ifndef DHEIGHTFIELD_CORNER_ORIGIN
        else
        {
            for ( int i = 0; i < numTerrainContacts; ++i )
            {
                pContact = CONTACT(contact,i*skip);
                pContact->pos[ 0 ] -= HalfWidthX;
                pContact->pos[ 1 ] -= HalfWidthY;
            }
        }
#endif // !DHEIGHTFIELD_CORNER_ORIGIN
    }
    // Return contact count.    
    printf("num collided------------- %d\n", numTerrainContacts);
     return numTerrainContacts;   

}


void MLSCollision::getAABB (dGeomID o, dReal aabb[6], const boost::shared_ptr<envire::MLSGrid>& mls){	

 
           MinHeight = -2.0;
           MaxHeight =  2.0;
           widthX  = mls->getCellSizeX()*mls->getScaleX();
           widthY  = mls->getCellSizeY()*mls->getScaleY();
           bool gflags = true;
           bool WrapMode = false;
           
		   HalfWidthX = widthX/REAL(2.0);
		   HalfWidthY = widthY/REAL(2.0);
                          
    if ( WrapMode == 0 )
    {
        // Finite
        if ( gflags & GEOM_PLACEABLE )
        {
            dReal dx[6], dy[6], dz[6];

            // Y-axis
            if (MinHeight != -dInfinity)
            {
                dz[0] = ( o->final_posr->R[ 2]  * MinHeight );
                dz[1] = ( o->final_posr->R[ 6]  * MinHeight );
                dz[2] = ( o->final_posr->R[ 10] * MinHeight );
            }
            else
            {
                // Multiplication is performed to obtain infinity of correct sign
                dz[0] = ( o->final_posr->R[ 2]  ? o->final_posr->R[ 2]  * -dInfinity : REAL(0.0) );
                dz[1] = ( o->final_posr->R[ 6]  ? o->final_posr->R[ 6]  * -dInfinity : REAL(0.0) );
                dz[2] = ( o->final_posr->R[ 10] ? o->final_posr->R[ 10] * -dInfinity : REAL(0.0) );
            }

            if (MaxHeight != dInfinity)
            {
                dz[3] = ( o->final_posr->R[ 2] * MaxHeight );
                dz[4] = ( o->final_posr->R[ 6] * MaxHeight );
                dz[5] = ( o->final_posr->R[ 10] * MaxHeight );
            }
            else
            {
                dz[3] = ( o->final_posr->R[ 2] ? o->final_posr->R[ 2] * dInfinity : REAL(0.0) );
                dz[4] = ( o->final_posr->R[ 6] ? o->final_posr->R[ 6] * dInfinity : REAL(0.0) );
                dz[5] = ( o->final_posr->R[ 10] ? o->final_posr->R[ 10] * dInfinity : REAL(0.0) );
            }

#ifdef DHEIGHTFIELD_CORNER_ORIGIN

            // X-axis
            dx[0] = 0;	dx[3] = ( o->final_posr->R[ 0] * widthX );
            dx[1] = 0;	dx[4] = ( o->final_posr->R[ 4] * widthX );
            dx[2] = 0;	dx[5] = ( o->final_posr->R[ 8] * widthX );

            // Y-axis
            dy[0] = 0;	dy[3] = ( o->final_posr->R[ 1] * widthY );
            dy[1] = 0;	dy[4] = ( o->final_posr->R[ 5] * widthY );
            dy[2] = 0;	dy[5] = ( o->final_posr->R[ 9] * widthY );

#else // DHEIGHTFIELD_CORNER_ORIGIN

            // X-axis
            dx[0] = ( o->final_posr->R[ 0] * -HalfWidthX );
            dx[1] = ( o->final_posr->R[ 4] * -HalfWidthX );
            dx[2] = ( o->final_posr->R[ 8] * -HalfWidthX );
            dx[3] = ( o->final_posr->R[ 0] * HalfWidthX );
            dx[4] = ( o->final_posr->R[ 4] * HalfWidthX );
            dx[5] = ( o->final_posr->R[ 8] * HalfWidthX );

            // Y-axis
            dy[0] = ( o->final_posr->R[ 1] * -HalfWidthY );
            dy[1] = ( o->final_posr->R[ 5] * -HalfWidthY);
            dy[2] = ( o->final_posr->R[ 9] * -HalfWidthY );
            dy[3] = ( o->final_posr->R[ 1] * HalfWidthY );
            dy[4] = ( o->final_posr->R[ 5] * HalfWidthY );
            dy[5] = ( o->final_posr->R[ 9] * HalfWidthY );

#endif // DHEIGHTFIELD_CORNER_ORIGIN

            // X extents
            aabb[0] = o->final_posr->pos[0] +
                dMIN3( dMIN( dx[0], dx[3] ), dMIN( dy[0], dy[3] ), dMIN( dz[0], dz[3] ) );
            aabb[1] = o->final_posr->pos[0] +
                dMAX3( dMAX( dx[0], dx[3] ), dMAX( dy[0], dy[3] ), dMAX( dz[0], dz[3] ) );

            // Y extents
            aabb[2] = o->final_posr->pos[1] +
                dMIN3( dMIN( dx[1], dx[4] ), dMIN( dy[1], dy[4] ), dMIN( dz[1], dz[4] ) );
            aabb[3] = o->final_posr->pos[1] +
                dMAX3( dMAX( dx[1], dx[4] ), dMAX( dy[1], dy[4] ), dMAX( dz[1], dz[4] ) );

            // Z extents
            aabb[4] = o->final_posr->pos[2] +
                dMIN3( dMIN( dx[2], dx[5] ), dMIN( dy[2], dy[5] ), dMIN( dz[2], dz[5] ) );
            aabb[5] = o->final_posr->pos[2] +
                dMAX3( dMAX( dx[2], dx[5] ), dMAX( dy[2], dy[5] ), dMAX( dz[2], dz[5] ) );
        }
        else
        {

#ifdef DHEIGHTFIELD_CORNER_ORIGIN

            aabb[0] = 0;					aabb[1] = widthX;
            aabb[2] = 0;					aabb[3] = widthY;
            aabb[4] = MinHeight;			aabb[5] = MaxHeight;

#else // DHEIGHTFIELD_CORNER_ORIGIN

            aabb[0] = -widthX/REAL(2.0);	aabb[1] = +widthX/REAL(2.0);
            aabb[2] = -widthY/REAL(2.0);	aabb[3] = +widthY/REAL(2.0);
            aabb[4] =  MinHeight;	   		aabb[5] =  MaxHeight;

#endif // DHEIGHTFIELD_CORNER_ORIGIN

        }
    }
    else
    {
        // Infinite
        if ( gflags & GEOM_PLACEABLE )
        {
            aabb[0] = -dInfinity;			aabb[1] = +dInfinity;
            aabb[2] = -dInfinity;			aabb[3] = +dInfinity;
            aabb[4] = -dInfinity;			aabb[5] = +dInfinity;
        }
        else
        {
            aabb[0] = -dInfinity;			aabb[1] = +dInfinity;
            aabb[2] = -dInfinity;			aabb[3] = +dInfinity;
            aabb[4] =  MinHeight;			aabb[5] =  MaxHeight;
        }
    }
          
                printf("aabb(%f %f)(%f %f)(%f %f)\n", aabb[0],aabb[1], aabb[2],aabb[3], aabb[4],aabb[5]);	        
           
}




