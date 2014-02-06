#include <string>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

class PropertyDetector
{
	protected:

    	pcl::PointCloud<pcl::PointXYZ>::Ptr objectCloudPtr;
        int objectId;

  	public:

        PropertyDetector(){};
        
    	void setPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr object_cloud_ptr)
        {
           objectCloudPtr = object_cloud_ptr;
        };

        void setObjectId(int object_id)
        {
            objectId = object_id;
        }

    	virtual void calculatePropertyValue()=0;


};