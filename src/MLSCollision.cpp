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
    
    unsigned int numCollidingCells = 0;
    //dContactGeom BoxContact[10];// = m_contacts;     
    dContactGeom *BoxContact = m_contacts;        
    boost::shared_ptr<MlsFieldRectangular> grid_cell(new MlsFieldRectangular);       
    boost::shared_ptr<CollidingCellGroup> grid_group(new CollidingCellGroup);
    
    std::cout << "numX numZ = " << numX << "," << numZ <<std::endl;
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
					grid_cell->vertices[0].vertex[0] = x * mls->getScaleX();
					grid_cell->vertices[0].vertex[1] = p.mean;
					grid_cell->vertices[0].vertex[2] = z * mls->getScaleY();					
	        }  
	       
	 	    for( MLSGrid::iterator cit = mls->beginCell(x+1,z); cit != mls->endCell(); cit++ )
		    {
				MLSGrid::SurfacePatch p( *cit );  
	            isBCollide = p.mean > minO2Height; 
					grid_cell->vertices[1].vertex[0] = x * mls->getScaleX();
					grid_cell->vertices[1].vertex[1] = p.mean;
					grid_cell->vertices[1].vertex[2] = z * mls->getScaleY();		            
					
	        }      
		    for( MLSGrid::iterator cit = mls->beginCell(x,z+1); cit != mls->endCell(); cit++ )
		    {
				MLSGrid::SurfacePatch p( *cit );  
	            isCCollide = p.mean > minO2Height;  
					grid_cell->vertices[2].vertex[0] = x * mls->getScaleX();
					grid_cell->vertices[2].vertex[1] = p.mean;
					grid_cell->vertices[2].vertex[2] = z * mls->getScaleY();					
	        }
	 	    for( MLSGrid::iterator cit = mls->beginCell(x+1,z+1); cit != mls->endCell(); cit++ )
		    {
				MLSGrid::SurfacePatch p( *cit );  
	            isDCollide = p.mean > minO2Height;
					grid_cell->vertices[3].vertex[0] = x * mls->getScaleX();
					grid_cell->vertices[3].vertex[1] = p.mean;
					grid_cell->vertices[3].vertex[2] = z * mls->getScaleY();					
	        }      
			if (isACollide || isBCollide || isCCollide || isDCollide)
			{  
					grid_group->rect[numCollidingCells++] = *grid_cell;
			}               
	       
		}
	}   
 
	    int maxBoxNum = 4;
	    double boxlength = 0.5;
	    
		dxBox* colliding_box[maxBoxNum];
		for(int i=0; i<4; i++) colliding_box[i] = new dxBox (0,1,1,1);   //TODE space pointer has to be added
      // dContactGeom cg[10];
		//   int collided = dCollideSphereBox (o2, colliding_box[0], flags, &cg[0], skip);	
		int collided = dCollideSphereBox (o2, colliding_box[0], flags, BoxContact, skip);			   
    
    for (unsigned int k = 0; k < numCollidingCells; k++)
    {
		for(unsigned int i=0;i<4;i++){   //we need to build 4 boxs per a rect
		   //set positions and size of Boxs A,B,C,D from collision
		   dVector3Copy(grid_group->rect[k].vertices[i].vertex, colliding_box[i]->final_posr->pos); 
		   colliding_box[i]->side[0] = grid_group->rect[k].vertices[1].vertex[0]-grid_group->rect[k].vertices[0].vertex[0];
		   colliding_box[i]->side[1] = boxlength;	
		   colliding_box[i]->side[2] = grid_group->rect[k].vertices[2].vertex[2]-grid_group->rect[k].vertices[0].vertex[2];	
		   colliding_box[i]->final_posr->pos[0] -= (colliding_box[i]->side[0]/2);   
		   colliding_box[i]->final_posr->pos[1] = colliding_box[i]->final_posr->pos[1] - boxlength/2;  	
		   colliding_box[i]->final_posr->pos[2] -= (colliding_box[i]->side[2]/2); 	
		   	
		   printf("(k,i)=(%d %d) pos(%f %f %f)\n",k,i,colliding_box[i]->final_posr->pos[0],colliding_box[i]->final_posr->pos[1]
					,colliding_box[i]->final_posr->pos[0]);		   		   	   
		//   int collided = dCollideSphereBox (o2, colliding_box[i], flags, BoxContact, skip);
	   

		   //if(collided && numTerrainContacts < 4) {	   
  		       //pContact = CONTACT(contact, numTerrainContacts*skip);			   
		       //dVector3Copy(BoxContact->pos, pContact->pos);
               ////create contact using Plane Normal
               //dOPESIGN(pContact->normal, =, -, BoxContact->normal);	                      
		       //pContact->depth = BoxContact->depth;	
       
 	           //numTerrainContacts++;	
		   //} 
		   
		}

	} 
	for(int i=0; i<maxBoxNum; i++) delete colliding_box[i];   
	printf("dCollideMlsfieldZone.......numTerrainContacts = %d \n",numTerrainContacts);
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
     std::cout << "\n ...REAL 0.1 nMinX, nMaxX, nMinZ, nMaxZ : " << nMinX <<","<< nMaxX <<","<< nMinZ <<","<< nMaxZ<<std::endl;    
       std::cout << "\n ...Height =  " << mls->getHeight()<<std::endl;   
       

       
       
       
         
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
