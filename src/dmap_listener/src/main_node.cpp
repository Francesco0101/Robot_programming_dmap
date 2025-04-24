#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

#include "dmap.h"
#include "draw_helpers.h"
#include "dmap_localizer.h"

#include <limits>  // Per std::isfinite

ros::Publisher pose_pub;

GridMapping grid_mapping;
DMapLocalizer localizer;
std::vector<Vector2f> obstacles;

bool map_received = false;
bool init_received = false;

float resolution;
float max_range = 10.0f;
float expansion_range = 1.0f;

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    resolution = msg->info.resolution;
    Vector2f origin(msg->info.origin.position.x, msg->info.origin.position.y);
    grid_mapping.reset(origin, resolution);

    uint32_t width = msg->info.width;
    uint32_t height = msg->info.height;

    obstacles.clear();  // Pulizia della lista per evitare duplicazioni

    for (uint32_t i = 0; i < height; ++i)
    {
        for (uint32_t j = 0; j < width; ++j)
        {
            int index = i * width + j;
            int value = msg->data[index];
            if (value >= 50)  // Soglia più flessibile
            {
                Vector2f coord = grid_mapping.grid2world(Vector2f(j, i));
                obstacles.push_back(coord);
            }
        }
    }

    localizer.setMap(obstacles, resolution, 10);
    map_received = true;
    ROS_INFO("Map received and processed. Obstacles: %lu", obstacles.size());
}

void initCallback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
    Vector2f position(msg.pose.pose.position.x, msg.pose.pose.position.y);
    float yaw = atan2(2.0 * (msg.pose.pose.orientation.w * msg.pose.pose.orientation.z +
                             msg.pose.pose.orientation.x * msg.pose.pose.orientation.y),
                      1.0 - 2.0 * (msg.pose.pose.orientation.y * msg.pose.pose.orientation.y +
                                   msg.pose.pose.orientation.z * msg.pose.pose.orientation.z));

    Isometry2f pose = Isometry2f::Identity();
    pose.translation() = position;
    pose.linear() = Eigen::Rotation2Df(yaw).toRotationMatrix();


    localizer.X = pose;
    init_received = true;

    ROS_INFO("Initial pose set at (%.2f, %.2f, %.2f)", position.x(), position.y(), yaw);
}

void scanCallback(const sensor_msgs::LaserScan& scan)
{
    if (!map_received || !init_received)
    {
        ROS_WARN_THROTTLE(1.0, "Waiting for map and initial pose...");
        return;
    }

    std::vector<Vector2f> endpoints;
    for (size_t i = 0; i < scan.ranges.size(); ++i)
    {
        float angle = scan.angle_min + i * scan.angle_increment;
        float range = scan.ranges[i];

        if (std::isfinite(range) && range >= scan.range_min && range <= scan.range_max)
        {
            endpoints.emplace_back(range * cos(angle), range * sin(angle));
        }
    }

    localizer.localize(endpoints, 10);

    Isometry2f pose = localizer.X;

    Eigen::Matrix3f R;
    R << pose.linear()(0, 0), pose.linear()(0, 1), 0,
         pose.linear()(1, 0), pose.linear()(1, 1), 0,
         0, 0, 1;

    Eigen::Quaternionf q(R);

    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();  // Aggiunta del timestamp
    odom.header.frame_id = "map";
    odom.child_frame_id = "robot";
    odom.pose.pose.position.x = pose.translation().x();
    odom.pose.pose.position.y = pose.translation().y();
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    pose_pub.publish(odom);

    std::cerr << "Estimated pose: " << pose.translation().transpose() << std::endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dmap_localizer_node");
    ros::NodeHandle nh("~");  // Handle privato per parametri

    nh.param("max_range", max_range, 10.0f);
    nh.param("expansion_range", expansion_range, 1.0f);

    pose_pub = nh.advertise<nav_msgs::Odometry>("/localization/odom", 10);
    ros::Subscriber map_sub = nh.subscribe("/map", 1, mapCallback);
    ros::Subscriber init_sub = nh.subscribe("/initialpose", 1, initCallback);
    ros::Subscriber scan_sub = nh.subscribe("/base_scan", 1, scanCallback);
    
    ros::Rate rate(50);  // Ciclo a 50 Hz
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
