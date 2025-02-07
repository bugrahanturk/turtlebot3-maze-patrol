#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tuw_multi_robot_msgs/Graph.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <vector>
#include <set>
#include <stack>
#include <cmath>

//!< Vertex Yapisi
struct Vertex 
{
    double x, y;
};

//!< Global Degiskenler
tuw_multi_robot_msgs::Graph current_graph;  //!< Mevcut graph
nav_msgs::OccupancyGrid map_data;           //!< Harita bilgisi
Vertex robot_position;                      //!< Robotun pozisyonu
bool robot_position_initialized = false;    //!< Robot pozisyonunun baslangic durumu
bool map_initialized = false;               //!< Harita yuklendi mi kontrolu
bool graph_received = false;                //!< Graph okundu mu kontrolu
std::set<size_t> visited_vertices;          //!< Ziyaret edilen dugumler
std::set<size_t> visited_cells;             //!< Ziyaret edilen hucelerin indeksleri
std::stack<size_t> traversal_stack;         //!< Backtracking icin stack
double total_distance     = 0.0;            //!< Toplam alinan mesafe
double total_covered_area = 0.0;            //!< Toplam kapsanan alan
bool goal_reached         = true;           //!< Hedefe ulasildi mi

//!< Oklid Mesafesi Hesaplama
double Calculate_Distance(const Vertex& a, const Vertex& b) 
{
    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
}

size_t Find_Nearest_Vertex(const Vertex& position) 
{
    size_t nearest_index = 0;
    double min_distance  = std::numeric_limits<double>::max();

    for (size_t i = 0; i < current_graph.vertices.size(); ++i) 
    {
        const auto& vertex = current_graph.vertices[i];
        Vertex vertex_position{vertex.path.front().x, vertex.path.front().y};
        double distance = Calculate_Distance(position, vertex_position);
        
        if (distance < min_distance) 
        {
            min_distance = distance;
            nearest_index = i;
        }
    }

    return nearest_index;
}

//!< Pozisyonu Harita Gridine Donusturme
size_t Position_To_Index(const Vertex& position) 
{
    int map_x = static_cast<int>((position.x - map_data.info.origin.position.x) / map_data.info.resolution);
    int map_y = static_cast<int>((position.y - map_data.info.origin.position.y) / map_data.info.resolution);

    if (map_x < 0 || map_y < 0 || map_x >= map_data.info.width || map_y >= map_data.info.height) 
    {
         //!< Harita Disi
        return std::numeric_limits<size_t>::max();
    }

    return map_y * map_data.info.width + map_x;
}

//!< Robot Pozisyonu Callback
void Odometry_Callback(const nav_msgs::Odometry::ConstPtr& odom_msg) 
{
    robot_position.x            = odom_msg->pose.pose.position.x;
    robot_position.y            = odom_msg->pose.pose.position.y;
    robot_position_initialized  = true;
}

//!< Harita Callback
void Map_Callback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg) 
{
    map_data        = *map_msg;
    map_initialized = true;
    ROS_INFO("Harita alindi: %zu x %zu", map_data.info.width, map_data.info.height);
}

//!< Graph Callback
void Graph_Callback(const tuw_multi_robot_msgs::Graph::ConstPtr& graph_msg) 
{
    if(!graph_received)
    {
        current_graph = *graph_msg;
        graph_received = true;
        ROS_INFO("Graph Alindi. Graphtaki Vertex Sayisi : %zu", current_graph.vertices.size());
    }
}

//!< Goal reached kontrolu
void Status_Callback(const actionlib_msgs::GoalStatusArray::ConstPtr& status_msg) 
{
    if (!status_msg->status_list.empty()) 
    {
        const auto& status = status_msg->status_list.back();
        goal_reached = (status.status == actionlib_msgs::GoalStatus::SUCCEEDED);
    }
}

//!< Hedef gonderimi
void Send_Goal(const Vertex& goal, ros::Publisher& goal_pub) 
{
    geometry_msgs::PoseStamped goal_msg;

    goal_msg.header.frame_id = "map";
    goal_msg.header.stamp = ros::Time::now();
    goal_msg.pose.position.x = goal.x;
    goal_msg.pose.position.y = goal.y;
    goal_msg.pose.orientation.w = 1.0;

    goal_pub.publish(goal_msg);
    ROS_INFO("Hedef gonderildi: (%f, %f)", goal.x, goal.y);
    goal_reached = false;
}

//!< Ziyaret Edilen Hucreleri Guncelleme
void Update_Coverage() 
{
    if (!map_initialized || !robot_position_initialized) 
    {
        return;
    }

    size_t cell_index = Position_To_Index(robot_position);
    
    if (cell_index != std::numeric_limits<size_t>::max() && visited_cells.find(cell_index) == visited_cells.end()) 
    {
        visited_cells.insert(cell_index);

        //!< Grid alaninin hesapla ve toplam kapsama alanina ekle
        double cell_area = std::pow(map_data.info.resolution, 2);
        total_covered_area += cell_area;
    }
}

//!< Kapsama Orani Hesaplama
double Calculate_Coverage_Percentage() 
{
    if (current_graph.vertices.empty())
     {
        ROS_WARN("Graph bos! Kapsama hesabi yapilamiyor.");
        return 0.0;
    }

    size_t total_vertices = current_graph.vertices.size();
    size_t visited_count  = visited_vertices.size();

    return (static_cast<double>(visited_count) / total_vertices) * 100.0;
}

void Traverse_Graph(ros::Publisher& goal_pub) 
{
    /**
    * Bu fonksiyon, bir graph üzerinde robotun devriye gezmesini sağlar. 
    * Robot, graph'taki düğümleri sırasıyla ziyaret eder ve tüm düğümleri kapsayacak şekilde hareket eder. 
    * Hedef olarak en yakın düğüme gitmeye çalışır ve tüm komşular ziyaret edildiğinde geri izleme (backtracking) 
    * yaparak başka bir düğüme geçer. Aynı zamanda robotun kapsadığı alanı günceller ve kapsama oranını hesaplar.
    */

    if (current_graph.vertices.empty()) 
    {
        ROS_WARN("Graph BOS geldi!!!");
        return;
    }

    size_t current_vertex = Find_Nearest_Vertex(robot_position);
    traversal_stack.push(current_vertex);

    while (!traversal_stack.empty()) 
    {
        if (goal_reached) 
        {
            current_vertex = traversal_stack.top();

            if (visited_vertices.find(current_vertex) == visited_vertices.end()) 
            {
                const auto& vertex = current_graph.vertices[current_vertex];
                Vertex vertex_position{vertex.path.front().x, vertex.path.front().y};

                Send_Goal(vertex_position, goal_pub);

                total_distance += Calculate_Distance(robot_position, vertex_position);
                robot_position = vertex_position;

                visited_vertices.insert(current_vertex);

                for (const auto& neighbor : vertex.successors) 
                {
                    if (visited_vertices.find(neighbor) == visited_vertices.end()) 
                    {
                        traversal_stack.push(neighbor);
                    }
                }
                for (const auto& neighbor : vertex.predecessors) 
                {
                    if (visited_vertices.find(neighbor) == visited_vertices.end()) 
                    {
                        traversal_stack.push(neighbor);
                    }
                }
            } 
            else 
            {
                traversal_stack.pop();
            }

            Update_Coverage();
            double coverage_percentage = Calculate_Coverage_Percentage();
            ROS_INFO("Anlik kapsama orani: %f%%, Toplam kapsanan alan: %f m^2", coverage_percentage, total_covered_area);
        }

        ros::spinOnce();
    }

    ROS_INFO("Graph gezinimi Tamamlandi. Toplam alinan mesafe: %f metre.", total_distance);
}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "voronoi_patrol_node");
    ros::NodeHandle nh;

    //!< Graph, odometry ve map abonelikleri
    ros::Subscriber graph_sub  = nh.subscribe("/segments", 1, Graph_Callback);
    ros::Subscriber odom_sub   = nh.subscribe("/odom", 1, Odometry_Callback);
    ros::Subscriber map_sub    = nh.subscribe("/map", 1, Map_Callback);
    ros::Subscriber status_sub = nh.subscribe("/move_base/status", 1, Status_Callback);

    //!< Hedef Yayinlayicisi
    ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);

    ros::Rate rate(1);
    while (ros::ok() && (!robot_position_initialized || current_graph.vertices.empty() || !map_initialized || !graph_received)) 
    {
        ROS_INFO("Graph, robot pozisyonu ve harita bekleniyor...");
        ros::spinOnce();
        rate.sleep();
    }

    Traverse_Graph(goal_pub);

    return 0;
}
