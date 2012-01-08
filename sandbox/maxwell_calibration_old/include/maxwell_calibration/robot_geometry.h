/* 
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

#include <pcl_ros/point_cloud.h>
#include <geometric_shapes/bodies.h>

class BaseGeometry{
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
    typedef pcl::PointXYZ Point;

  public:
    /* create instance, using a mesh file for the base */
    BaseGeometry(std::string mesh_file);

    /* compute point inclusion score for a cloud overlap with base */
    double computeScore(const PointCloud::ConstPtr &cloud);

  private:
    shapes::Mesh* createMeshFromBinaryStlData(const char *data, unsigned int size, float scale);

    bodies::ConvexMesh* scaled_up_mesh_;      // we want points to land on a thin crust
    bodies::ConvexMesh* scaled_down_mesh_;    //   between scaled up/down meshes
    bool configured_;
};


class HeadGeometry{
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
    typedef pcl::PointCloud<pcl::PointXYZNormal> NormalCloud;
    typedef pcl::PointXYZ Point;
    typedef pcl::PointXYZNormal PointNormal;

    typedef pcl::PassThrough<Point> PassThroughFilter;
    typedef pcl::NormalEstimation<Point,PointNormal> NormalEstimator;

  public: 
    HeadGeometry(double x, double y, double z);    

    /* compute score for a cloud */
    double computeScore(const PointCloud::ConstPtr &cloud);

  private:
    double limit_x_;
    double limit_y_;
    double limit_z_;
};

