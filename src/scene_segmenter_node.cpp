#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <boost/shared_ptr.hpp>
#include <pcl_ros/point_cloud.h>

#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"
#include "perception_msgs/SegmentedObject.h"
#include "perception_msgs/SegmentedObjectList.h"
#include "perception_msgs/ObjectCenterProperty.h"

#include "cluster_extractor.h"



namespace scene_segmenter_node
{
    class SceneSegmenterNode
    {
        private:
            ros::NodeHandle node_handle;

            ros::Subscriber pointCloudSubscriber;
            void pointCloudMessageCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);

            ros::Publisher segmentedObjectsPublisher;

        public:
            SceneSegmenterNode();
    };


    SceneSegmenterNode::SceneSegmenterNode(): node_handle("")
    {

        pointCloudSubscriber = node_handle.subscribe("point_cloud", 1000, &SceneSegmenterNode::pointCloudMessageCallback, this);

        segmentedObjectsPublisher = node_handle.advertise<perception_msgs::SegmentedObjectList>("segmented_objects",10);

        ROS_INFO("scene_segmenter_node ready");

        //demo
        pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
        pcl::PCDReader reader;
        std::string fileName = ros::package::getPath("object_models") + "/models/pcl_example_scenes/table_scene_lms400.pcd";
        reader.read (fileName, *cloud); 
        ClusterExtractor clusterExtractor = ClusterExtractor();
        clusterExtractor.setCloud(cloud);
        clusterExtractor.computeClusters();
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = clusterExtractor.getCloudClusters();


        //for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
        //{
         //   cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
        //}
            


        //segmentedObjectsPublisher.publish(segmentedObjects);
    }


    void SceneSegmenterNode::pointCloudMessageCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        ROS_INFO("Received point cloud to process");
    }
}



int main(int argc, char **argv) 
{

  ros::init(argc, argv, "scene_segmenter_node");
  ros::NodeHandle nh;

 // scene_segmenter_node::SceneSegmenterNode node;

  ros::spin();
  return 0;
}
