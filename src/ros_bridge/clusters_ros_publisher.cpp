#include "ros_bridge/clusters_ros_publisher.h"


#include <vector>
#include <string>

namespace depth_clustering {

using std::vector;
using std::string;
using std::map;

ClustersRosPublisher::ClustersRosPublisher(ros::NodeHandle& node_handle, const std::string& topic_clusters)
                                            : Receiver(), Sender(SenderType::STREAMER)
{
    clusters_publisher = node_handle.advertise<std_msgs::Float32MultiArray>(topic_clusters, 10);
}

void ClustersRosPublisher::OnNewObjectReceived(const std::unordered_map<uint16_t, Cloud>& clusters, 
                                                const int)
{
    time_utils::Timer timer;
    clusters_position.data.clear();
    for(const auto& cluster : clusters)
    {
        clusters_position.data.push_back(cluster.second.ComputeClusterCenter().x());
        clusters_position.data.push_back(cluster.second.ComputeClusterCenter().y());
        clusters_position.data.push_back(cluster.second.ComputeClusterCenter().z());
    }
    this->ShareDataWithAllClients(clusters);
    clusters_publisher.publish(clusters_position);
    fprintf(stderr, "INFO: publish clusters in: %lu us\n", timer.measure());
}

} 