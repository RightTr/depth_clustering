#ifndef SRC_ROS_BRIDGE_CLUSTERS_ROS_PUBLISHER_H_
#define SRC_ROS_BRIDGE_CLUSTERS_ROS_PUBLISHER_H_

#include <ros/ros.h>
#include "std_msgs/Float32MultiArray.h"

#include <string>
#include <map>

#include "communication/abstract_sender.h"
#include "utils/cloud.h"
#include "utils/timer.h"

namespace depth_clustering {

class ClustersRosPublisher : 
        public AbstractClient<std::unordered_map<uint16_t, Cloud>>,
        public AbstractSender<std::unordered_map<uint16_t, Cloud>>
{
    using Receiver = AbstractClient<std::unordered_map<uint16_t, Cloud>>;
    using Sender = AbstractSender<std::unordered_map<uint16_t, Cloud>>;

    public:
        ClustersRosPublisher(){}
        ClustersRosPublisher(ros::NodeHandle& node_handle, const std::string& topic_clusters);

        virtual ~ClustersRosPublisher() {}

        void OnNewObjectReceived(const std::unordered_map<uint16_t, Cloud>& clusters, 
                                const int) override;
    protected:

        ros::Publisher clusters_publisher;
        std_msgs::Float32MultiArray clusters_position;

};

}  

#endif  