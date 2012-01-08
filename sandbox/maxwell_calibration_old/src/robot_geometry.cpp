/* 
  Scoring for geometrical constructions
  Copyright (c) 2011 Michael Ferguson.  All right reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of the copyright holders nor the names of its 
        contributors may be used to endorse or promote products derived 
        from this software without specific prior written permission.
  
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL THE CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
  OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "maxwell_calibration/robot_geometry.h"
#include "resource_retriever/retriever.h"

/* Create instance, requires name of base mesh STL file */
BaseGeometry::BaseGeometry(std::string mesh_file):configured_(false)
{
    if (!mesh_file.empty())
    {
        resource_retriever::Retriever retriever;
        resource_retriever::MemoryResource res;

        try
        {
            res = retriever.get(mesh_file);
        }
        catch (resource_retriever::Exception& e)
        {
            ROS_ERROR("%s", e.what());    
            return;
        }

        if (res.size == 0)
            ROS_ERROR("Retrieved empty mesh for resource '%s'", mesh_file.c_str());
            return;
        }
        else
        {
            shapes::Shape up_mesh = createMeshFromBinaryStlData(reinterpret_cast<char*>(res.data.get()), res.size, 1.1);
            scaled_up_mesh_ = new bodies::ConvexMesh(up_mesh);

            shapes::Shape dn_mesh = createMeshFromBinaryStlData(reinterpret_cast<char*>(res.data.get()), res.size, 0.9);
            scaled_down_mesh_ = new bodies::ConvexMesh(dn_mesh);
        }
        configured_ = true;
    }
    else
    {
        ROS_ERROR("Mesh filename is empty.");
    }
}


/* Compute score based on how many points fall within scaled up mesh, but not in scaled down mesh */
double BaseGeometry::computeScore(const PointCloud::ConstPtr &cloud)
{
    if(!configured_)
    {
        ROS_ERROR("You must load a mesh before scoring.");
        return 0;
    }

    double score;

    for( int i = 0; i < cloud->points.size(); i++ )
    {
        Point p = cloud->points[i];
        btVector3 pt = btVector3( btScalar(p.x), btScalar(p.y), btScalar(p.y) );
        if(scaled_up_mesh.containsPoint(pt)){
            if(!scaled_down_mesh.containsPoint(pt)){
                score += 0.5;
            }else{
                score += -0.1;
            }
        }else{
            score += -0.1;
        }
    }
    ROS_INFO("BaseGeometry::computeScore: %f", (float) score );  
    return score;
}


/* This was pulled from shape_operations.cpp and a scale factor was added */
shapes::Mesh* BaseGeometry::createMeshFromBinaryStlData(const char *data, unsigned int size, float scale)
{
	const char* pos = data;
	pos += 80; // skip the 80 byte header
	
	unsigned int numTriangles = *(unsigned int*)pos;
	pos += 4;
	
	// make sure we have read enough data
	if ((long)(50 * numTriangles + 84) <= size)
	{
	    std::vector<btVector3> vertices;
	    
	    for (unsigned int currentTriangle = 0 ; currentTriangle < numTriangles ; ++currentTriangle)
	    {
		// skip the normal
		pos += 12;
		
		// read vertices 
		btVector3 v1(0,0,0);
		btVector3 v2(0,0,0);
		btVector3 v3(0,0,0);
		
		v1.setX((*(float*)pos)*scale);
		pos += 4;
		v1.setY((*(float*)pos)*scale);
		pos += 4;
		v1.setZ((*(float*)pos)*scale);
		pos += 4;
		
		v2.setX((*(float*)pos)*scale);
		pos += 4;
		v2.setY((*(float*)pos)*scale);
		pos += 4;
		v2.setZ((*(float*)pos)*scale);
		pos += 4;
		
		v3.setX((*(float*)pos)*scale);
		pos += 4;
		v3.setY((*(float*)pos)*scale);
		pos += 4;
		v3.setZ((*(float*)pos)*scale);
		pos += 4;
		
		// skip attribute
		pos += 2;
		
		vertices.push_back(v1);
		vertices.push_back(v2);
		vertices.push_back(v3);
	    }
	    
	    return createMeshFromVertices(vertices);
    }
    return NULL;
}


/* Constructor */
HeadGeometry::HeadGeometry(double x, double y, double z):limit_x_(x), limit_y_(y), limit_z_(z)
{
    // empty?
}   


/* Compute score based on how well the normals approximate the floor */
double HeadGeometry::computeScore(const PointCloud::ConstPtr &cloud)
{
    // temp clouds
    NormalCloud normals;
    PointCloud filtered_x;
    PointCloud filtered_y;
    PointCloud output;

    // filter in x,y,z directions
    PassThroughFilter pass;
    pass.setInputCloud(cloud);  // cloud should be in base_link frame!
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (-limit_x_, limit_x_);
    pass.filter(filtered_x);

    pass.setInputCloud (filtered_x.makeShared());
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (-limit_y_, limit_y_);
    pass.filter(filtered_y);

    pass.setInputCloud (filtered_y.makeShared());
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (-limit_z_, limit_z_);
    pass.filter(output);

    // normal estimation
    NormalEstimator estimator;
    estimator.setKSearch(25);
    estimator.setSearchMethod (boost::make_shared<pcl::KdTreeFLANN<Point> > ());
    estimator.setInputCloud (outputs.makeShared());
    //pcl::copyPointCloud (outputs, normals);
    estimator.compute (normals);
        
    // find consensus of normals
    if ( normals.points.size() > 0 )
    {
        float x = 0;
        float y = 0;
        float z = 0;
        for (size_t i = 0; i < normals.points.size() ; ++i)
        {
            x += normals.points[i].normal_x;
            y += normals.points[i].normal_y;
            z += normals.points[i].normal_z;
        }
        x = x/normals.points.size();
        y = y/normals.points.size();
        z = z/normals.points.size();
            
        btVector3 normal = btVector3( btScalar(x), btScalar(y), btScalar(z) );
        normal.normalize();
        ROS_INFO("HeadGeometry::computeScore, average normal: %f %f %f", normal.getX(), normal.getY(), normal.getZ() );   
    }

    // compute score
    return (double) btDistance(normal, btVector3( btScalar(0), btScalar(0), btScalar(1) ) );
}

