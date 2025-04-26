#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

#include "dmap.h"
#include "draw_helpers.h"
#include "dmap_localizer.h"

#include <limits>  

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

    // Pulisco la lista degli ostacoli da eventuali dati vecchi
    obstacles.clear();  

    for (uint32_t i = 0; i < height; ++i)
    {
        for (uint32_t j = 0; j < width; ++j)
        {
            int index = i * width + j;        
            int value = msg->data[index];     
            if (value >= 50)                    
            {
                ROS_INFO("Occupied cell at (%d, %d)", j, i);
                Vector2f coord = grid_mapping.grid2world(Vector2f(j, i));
                // Aggiungo la cella come ostacolo
                obstacles.push_back(coord);
            }
        }
    }

    // Passo gli ostacoli trovati al localizzatore
    localizer.setMap(obstacles, resolution, 10);

    map_received = true;
    ROS_INFO("Map received and processed. Obstacles: %lu", obstacles.size());
}

void initCallback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
    // Estraggo la posizione iniziale (x, y)
    Vector2f position(msg.pose.pose.position.x, msg.pose.pose.position.y);

    // Estraggo i singoli componenti del quaternione
    float qx = msg.pose.pose.orientation.x;
    float qy = msg.pose.pose.orientation.y;
    float qz = msg.pose.pose.orientation.z;
    float qw = msg.pose.pose.orientation.w;

    // Calcolo i termini intermedi
    float siny_cosp = 2.0 * (qw * qz + qx * qy);
    float cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);

    // Calcolo yaw (rotazione attorno all'asse Z)
    float yaw = atan2(siny_cosp, cosy_cosp);

    // Creo una trasformazione 2D (isometria) identità
    Isometry2f pose = Isometry2f::Identity();
    
    // Imposto la traslazione (x, y)
    pose.translation() = position;
    
    // Imposto la rotazione (solo in piano)
    pose.linear() = Eigen::Rotation2Df(yaw).toRotationMatrix();

    // Salvo la posa nel localizzatore
    localizer.X = pose;

    init_received = true;

    ROS_INFO("Initial pose set at (%.2f, %.2f, %.2f)", position.x(), position.y(), yaw);
}

void scanCallback(const sensor_msgs::LaserScan& scan)
{
    // Verifico di avere sia la mappa che la posa iniziale
    if (!map_received || !init_received)
    {
        ROS_WARN_THROTTLE(1.0, "Waiting for map and initial pose...");
        return;
    }

    std::vector<Vector2f> endpoints;

    // Scorro tutte le misure del laser
    for (size_t i = 0; i < scan.ranges.size(); ++i)
    {
        float angle = scan.angle_min + i * scan.angle_increment;
        float range = scan.ranges[i];

        // Se il range e' valido
        if (std::isfinite(range) && range >= scan.range_min && range <= scan.range_max)
        {
            // Converto da polari a cartesiane
            endpoints.emplace_back(range * cos(angle), range * sin(angle));
        }
    }

    localizer.localize(endpoints, 10);

    // Nuova posa stimata
    Isometry2f pose = localizer.X;

    // Creo una matrice di rotazione 3x3 per creare il quaternione (la parte 2x2 e' una rotazione sull'asse z che ci interessa, il resto serve per definire un quaternione)
    Eigen::Matrix3f R;
    R << pose.linear()(0, 0), pose.linear()(0, 1), 0,
         pose.linear()(1, 0), pose.linear()(1, 1), 0,
         0,                  0,                  1;
    Eigen::Quaternionf q(R);

    // Creo il messaggio di Odometry
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();  
    odom.header.frame_id = "map";          
    odom.child_frame_id = "robot";         

    // Imposto la posizione stimata
    odom.pose.pose.position.x = pose.translation().x();
    odom.pose.pose.position.y = pose.translation().y();
    odom.pose.pose.position.z = 0.0;

    // Imposto l'orientamento stimato
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    // Se fatto correttamente x e y dovrebbero essere prossimi allo 0
    std::cerr << "odom orient x: " << odom.pose.pose.orientation.x << std::endl;
    std::cerr << "odom orient y: " << odom.pose.pose.orientation.y << std::endl;

  
    pose_pub.publish(odom);

    std::cerr << "Estimated pose: " << pose.translation().transpose() << std::endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dmap_localizer_node");
    ros::NodeHandle nh("~");  
    nh.param("max_range", max_range, 10.0f);
    nh.param("expansion_range", expansion_range, 1.0f);

    pose_pub = nh.advertise<nav_msgs::Odometry>("/localization/odom", 10);
    ros::Subscriber map_sub = nh.subscribe("/map", 1, mapCallback);
    ros::Subscriber init_sub = nh.subscribe("/initialpose", 1, initCallback);
    ros::Subscriber scan_sub = nh.subscribe("/base_scan", 1, scanCallback);
    
    while (ros::ok()) {
        ros::spinOnce();
    }

    return 0;
}
