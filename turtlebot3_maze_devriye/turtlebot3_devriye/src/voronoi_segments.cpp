#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tuw_multi_robot_msgs/Graph.h>

ros::Publisher marker_pub;

//!< Segmentleri Marker olarak yayinlama
void Visualize_Segments(const tuw_multi_robot_msgs::Graph::ConstPtr& graph_msg) 
{
    visualization_msgs::Marker line_strip;

    line_strip.header.frame_id      = "map";
    line_strip.header.stamp         = ros::Time::now();
    line_strip.ns                   = "voronoi_segments";
    line_strip.action               = visualization_msgs::Marker::ADD;
    line_strip.pose.orientation.w   = 1.0;
    line_strip.id                   = 0;
    line_strip.type                 = visualization_msgs::Marker::LINE_LIST;
    line_strip.scale.x              = 0.05; 
    line_strip.color.r              = 1.0;
    line_strip.color.a              = 1.0;

    for (const auto& vertex : graph_msg->vertices) 
    {
        for (size_t i = 0; i < vertex.path.size() - 1; ++i) 
        {
            geometry_msgs::Point p1;
            p1.x = vertex.path[i].x;
            p1.y = vertex.path[i].y;
            p1.z = 0.0;

            geometry_msgs::Point p2;
            p2.x = vertex.path[i + 1].x;
            p2.y = vertex.path[i + 1].y;
            p2.z = 0.0;

            line_strip.points.push_back(p1);
            line_strip.points.push_back(p2);
        }
    }
    //!< yayinlayalim
    marker_pub.publish(line_strip);
}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "visualize_voronoi_segments");
    ros::NodeHandle nh;

    //!< Marker Publisher
    marker_pub = nh.advertise<visualization_msgs::Marker>("voronoi_segments_marker", 1);

    //!< Segment Subscriber
    ros::Subscriber graph_sub = nh.subscribe("/segments", 1, Visualize_Segments);

    ros::spin();
    return 0;
}
