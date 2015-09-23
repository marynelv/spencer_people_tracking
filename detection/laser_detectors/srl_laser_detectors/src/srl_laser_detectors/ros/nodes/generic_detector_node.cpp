#include <srl_laser_detectors/detector_factory.h>
#include <srl_laser_detectors/ros/ros_interface.h>


using namespace srl_laser_detectors;

/// Generic detector node which spawns a detector, specified by its name on the command line, using the DetectorFactory.
int main(int argc, char **argv)
{
    ros::init(argc, argv, "laser_detector");
    ros::NodeHandle nodeHandle("");
    ros::NodeHandle privateHandle("~");
    
    // Look up detector type
    DetectorFactory::init();  
    DetectorType type; privateHandle.param<std::string>("type", type, "");
    boost::shared_ptr<Detector> detector = DetectorFactory::createDetector(type, nodeHandle, privateHandle);

    // Other params
    std::string laser_topic = "laser";
    privateHandle.param<std::string>("laser_topic", laser_topic, laser_topic);

    std::string segmentation_topic = "segmentation_topic";
    privateHandle.param<std::string>("segmentation_topic", segmentation_topic, segmentation_topic);

    // Subscribe to laser scans and segmentations
    ROSInterface rosInterface(nodeHandle, privateHandle);
    rosInterface.connect(detector.get(), laser_topic, segmentation_topic);
    ros::spin();
}
