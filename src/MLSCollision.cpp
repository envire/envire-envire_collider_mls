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
//#define DEBUG_AABB
//#define DEBUG_CONTACT_POS

using namespace envire::collision;

int MLSCollision::dCollideSphereMls( const boost::shared_ptr<maps::grid::MLSMapKalman>& mls,
										   const int minX, const int maxX, const int minY, const int maxY, 
                                           dxGeom* o2, int flags, dContactGeom* contact, 
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
    
#ifdef DEBUG_CONTACT_POS				
			printf("(maxX , minX):(%d %d), (maxY , minY):(%d %d)\n",maxX , minX , maxY , minY);
#endif 
  
    unsigned int alignedNumRect = AlignBufferSize(numRectMax, TEMP_RECTANGULAR_BUFFER_ELEMENT_COUNT_ALIGNMENT);
    MlsFieldRectangular *rects = new MlsFieldRectangular[alignedNumRect]; 
    MlsFieldRectangular rect;

  	bool isCollide[4];
  	for(int i = 0; i<4; i++) isCollide[i] = false;
  	
	for ( x = minX, x_local = 0; x_local < numX; x++, x_local++)    
    {
		for ( y = minY, y_local = 0; y_local < numY; y++, y_local++) 	
		{
            for( int y1 = 0 ; y1<2; y1++) {
            for( int x1 = 0 ; x1<2; x1++) {
			   
			    maps::grid::MLSMapKalman::CellType::iterator patch = mls->at(x+x1,y+y1).begin();
				if (patch == mls->at(x+x1,y+y1).end()) {
							rect.vertices[x1+y1*2].vertex[0] = (x+x1) * mls->getResolution().x();
							rect.vertices[x1+y1*2].vertex[1] = (y+y1) * mls->getResolution().y();
							rect.vertices[x1+y1*2].is = false;	 
				}
				else{
					    for(patch = mls->at(x+x1,y+y1).begin(); patch != mls->at(x+x1,y+y1).end(); patch++ )
					    {
				            isCollide[x1+y1*2] = patch->getMean() > minO2Height;  
								rect.vertices[x1+y1*2].vertex[0] = (x+x1) * mls->getResolution().x();
								rect.vertices[x1+y1*2].vertex[1] = (y+y1) * mls->getResolution().y();	 
								rect.vertices[x1+y1*2].vertex[2] = patch->getMean();
								rect.vertices[x1+y1*2].is = true;	
									
			                maxZ = dMAX(maxZ, patch->getMean());
			                minZ = dMIN(minZ, patch->getMean());            
							//	 printf("(%fl) ",patch->mean);
				        }   
			    } 
		    }}
		
			if (isCollide[0] || isCollide[1] || isCollide[2] || isCollide[3])
			{  
				rects[numCollidingRects++] = rect;		
			}				
		
		}
	}   

        if (minO2Height - maxZ > -dEpsilon ) 
        {
            //totally above Mlsfield
#ifdef DEBUG_CONTACT_POS				
			printf("tottaly above mls--> minO2Height,maxZ:break (%f %f)\n",minO2Height,maxZ);
#endif  			                
            return 0;
        }
        
        if (minZ - maxO2Height > -dEpsilon )
        {
            // totally under Mlsfield
            pContact = CONTACT(contact, 0);	

            pContact->pos[0] = o2->final_posr->pos[0];
            pContact->pos[1] = o2->final_posr->pos[1];            
            pContact->pos[2] = minZ;

            pContact->normal[0] = 0;
            pContact->normal[1] = 0;
            pContact->normal[2] = -1;

            pContact->depth =  minZ - maxO2Height;

            pContact->side1 = -1;
            pContact->side2 = -1;
            
#ifdef DEBUG_CONTACT_POS				
			printf("tottaly under mls--> maxO2Height,minZ : (%f %f)\n",maxO2Height,minZ);
#endif   
			   
            return 0;
        }
        
	int maxBoxNum = 4;
	int maxSphereBoxContact = 1;
	dxBox* colliding_box[maxBoxNum];
    
	for(int i=0; i<4; i++) colliding_box[i] = new dxBox (0,1,1,1);   //TODE space pointer has to be added
    
    for (unsigned int k = 0; k < numCollidingRects; k++)
    {
		for(unsigned int i=0;i<4;i++){   //we need to build 4 boxs per a rect (k*4 loops)

			if(rect.vertices[i].is == false) rect.vertices[i].vertex[2] = minZ;  //in case that mls has no value
			
		   //set positions and size of Boxs A,B,C,D from collision
		   dVector3Copy(rects[k].vertices[i].vertex, colliding_box[i]->final_posr->pos); 
		   colliding_box[i]->side[0] = mls->getResolution().x();  
		   colliding_box[i]->side[1] = mls->getResolution().y();  
		   colliding_box[i]->side[2] = rects[k].vertices[i].vertex[2]; //boxlength;	

		   colliding_box[i]->final_posr->pos[0] -= (colliding_box[i]->side[0]/2);   
		   colliding_box[i]->final_posr->pos[1] -= (colliding_box[i]->side[1]/2); 	
		   colliding_box[i]->final_posr->pos[2] /= 2;   	

		int collided = dCollideSphereBox (o2, colliding_box[i], maxSphereBoxContact, &mls_contacts, skip);			
		   if(collided && (numTerrainContacts < flags)) {	  			   
  		       pContact = CONTACT(contact, numTerrainContacts*skip);	
  		       // given a pointer `contact' to a dContactGeom, return the dContactGeom at contact + numTerrainContacts*skip bytes.		   
		       dVector3Copy(mls_contacts.pos, pContact->pos);
               //create contact using Plane Normal
               dOPESIGN(pContact->normal, =, -, mls_contacts.normal);	                      
		       pContact->depth = mls_contacts.depth;	
    
            pContact->side1 = -1;
            pContact->side2 = -1;
            
#ifdef DEBUG_CONTACT_POS				
			 printf("++collided++, (i k)(%d %d), numContacts(%d), p(%f, %f, %f), n(%f %f %f), d(%f) \n",i,k,
			 numTerrainContacts,pContact->pos[0],pContact->pos[1],pContact->pos[2],
			 pContact->normal[0],pContact->normal[1],pContact->normal[2],pContact->depth);
#endif 			 			
			numTerrainContacts++;	 
		   } 
		}


	} 
	for(int i=0; i<maxBoxNum; i++) delete colliding_box[i];   
	delete[] rects;  
	
	return numTerrainContacts;

}


int MLSCollision::collide(dGeomID o1, dGeomID o2, int flags, dContactGeom* contact, int skip, const boost::shared_ptr< maps::grid::MLSMapKalman >& mls, int o2_class_id)
{

	dIASSERT( skip >= (int)sizeof(dContactGeom) );
    dIASSERT( o1->type == 14 );
    dIASSERT((flags & NUMC_MASK) >= 1);
    
           widthX  = mls->getSize().x();
           widthY  = mls->getSize().y();
           
		   HalfWidthX = widthX/REAL(2.0);
		   HalfWidthY = widthY/REAL(2.0);

    dVector3 posbak;
    dMatrix3 Rbak;
    dReal aabbbak[6];
    int gflagsbak;
    dVector3 pos0,pos1;
    dMatrix3 R1;

    int numTerrainContacts = 0;
    int numTerrainOrigContacts = 0;

#ifndef DHEIGHTFIELD_CORNER_ORIGIN

    const bool reComputeAABB = true;
#else
    const bool reComputeAABB = ( terrain->gflags & GEOM_PLACEABLE ) ? true : false;
#endif 

    if (reComputeAABB)
    {
        // Backup original o2 position, rotation and AABB.
        dVector3Copy( o2->final_posr->pos, posbak );
        dMatrix3Copy( o2->final_posr->R, Rbak );
        memcpy( aabbbak, o2->aabb, sizeof( dReal ) * 6 );
        gflagsbak = o2->gflags;
    }
	

        // Transform o2 into mls space.
        dSubtractVectors3( pos0, o2->final_posr->pos, o1->final_posr->pos );
        dMultiply1_331( pos1, o1->final_posr->R, pos0 );
        dMultiply1_333( R1, o1->final_posr->R, o2->final_posr->R );

        // Update o2 with transformed position and rotation.
        dVector3Copy( pos1, o2->final_posr->pos );
        dMatrix3Copy( R1, o2->final_posr->R );

          
#ifndef DHEIGHTFIELD_CORNER_ORIGIN
    o2->final_posr->pos[ 0 ] += HalfWidthX;
    o2->final_posr->pos[ 1 ] += HalfWidthY;
#endif 
		
    // Rebuild AABB for O2
    if (reComputeAABB)
        o2->computeAABB();
        
    //This has to be true in case to use infinite surface 
	const bool wrapped = WrapMode != 0;
	
	//if o2 is not overlaid in x-y plane, we will pass by the collision function 
	if ( !wrapped )
	{
		if (o2->aabb[0] > widthX || o2->aabb[2] > widthY) goto dCollideMlsExit;
		if (o2->aabb[1] < 0 || o2->aabb[3] < 0) goto dCollideMlsExit;
    }	
    
	{
        const dReal fInvScaleX = REAL(1.0) / mls->getResolution().x();
        int nMinX = (int)dFloor(dNextAfter(o2->aabb[0] * fInvScaleX, -dInfinity));
        int nMaxX = (int)dCeil(dNextAfter(o2->aabb[1] * fInvScaleX, dInfinity));
        
        const dReal fInvScaleY = REAL( 1.0 ) / mls->getResolution().y();
        int nMinY = (int)dFloor(dNextAfter(o2->aabb[2] * fInvScaleY, -dInfinity));
        int nMaxY = (int)dCeil(dNextAfter(o2->aabb[3] * fInvScaleY, dInfinity));   

		//select overlabing area between o1 and o2
		nMinX = dMAX( nMinX, 0 );
		nMaxX = dMIN( nMaxX, (int)(mls->getNumCells().x() - 1));  
		nMinY = dMAX( nMinY, 0 );
		nMaxY = dMIN( nMaxY, (int)(mls->getNumCells().y() - 1));
 
		dIASSERT ((nMinX < nMaxX) && (nMinY < nMaxY));

		int numMaxTerrainContacts = flags;    
 	       
        numTerrainContacts += dCollideSphereMls(mls,
            nMinX,nMaxX,nMinY,nMaxY,o2,flags,CONTACT(contact,numTerrainContacts*skip),skip	);
    
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
    
dCollideMlsExit:
    if (reComputeAABB)
    {
        // Restore o2 position, rotation and AABB
        dVector3Copy( posbak, o2->final_posr->pos );
        dMatrix3Copy( Rbak, o2->final_posr->R );
        memcpy( o2->aabb, aabbbak, sizeof(dReal)*6 );
        o2->gflags = gflagsbak;
        
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
#endif 

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
#endif 
    }
    // Return contact count.    
    return numTerrainContacts;   
}


void MLSCollision::getAABB (dGeomID o, dReal aabb[6], const boost::shared_ptr<maps::grid::MLSMapKalman>& mls)
{	
           MinHeight = -2.0;   //this value is the same value as the default height when creating heightfield in NodePhysics.cpp in MARS
           MaxHeight =  2.0;
           widthX  = mls->getSize().x();//mls->getNumCells().x()*mls->getResolution().x();
           widthY  = mls->getSize().y();//mls->getNumCells().y()*mls->getResolution().y();
           bool gflags = true;
           bool WrapMode = false;  //This has to be true in case to use infinite surface
           
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
#ifdef DEBUG_AABB				
	 printf(".aabb[0-5](%lf %lf %lf %lf %lf %lf)\n", aabb[0],aabb[1],aabb[2],aabb[3],aabb[4],aabb[5]);
#endif   

}




