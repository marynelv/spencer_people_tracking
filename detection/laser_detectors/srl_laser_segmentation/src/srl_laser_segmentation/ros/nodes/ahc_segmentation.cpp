#include <srl_laser_segmentation/ahc.h>
#include <srl_laser_segmentation/ros/ros_interface.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "ahc_segmentation");
    ros::NodeHandle nodeHandle("");
    ros::NodeHandle privateHandle("~");

    // Read parameters
    std::string laser_topic = "laser";
    privateHandle.param<std::string>("laser_topic", laser_topic, laser_topic);

    std::string segmentation_topic = "laser_segmentation";
    privateHandle.param<std::string>("segmentation_topic", segmentation_topic, segmentation_topic);

    double distanceThreshold; 
    privateHandle.param<double>("distance_threshold", distanceThreshold, 0.4); // in meters
    
    std::string linkageName; 
    privateHandle.param<std::string>("linkage", linkageName, "single");
    
    bool verbose = false; 
    privateHandle.param<bool>("verbose", verbose, verbose);

    // Set up the ROS interface, which also establishes the connection to the parameter server
    srl_laser_segmentation::ROSInterface rosInterface(nodeHandle, privateHandle, verbose);

    srl_laser_segmentation::EfficientAHC::Linkage linkage;
    if(linkageName == "average") {
        linkage = srl_laser_segmentation::EfficientAHC::AVERAGE_CPU;
    }
    else if(linkageName == "complete") {
        linkage = srl_laser_segmentation::EfficientAHC::COMPLETE;
    }
    else {
        linkage = srl_laser_segmentation::EfficientAHC::SINGLE;
    }

    // Initialize segmentation algorithm and connect to the ROS interface
    srl_laser_segmentation::AgglomerativeHierarchicalClustering segmentation(linkage, distanceThreshold,verbose);    

    // Subscribe to laser scans and publish segmentations
    rosInterface.connect(&segmentation, laser_topic, segmentation_topic);
    ros::spin();
}
