#include "truck_gazebo_plugins/occupancy_grid.h"

#include <gazebo_ros/conversions/builtin_interfaces.hpp>

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <vector>
#include <iostream>

namespace gazebo {

void OccupancyGridPlugin::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf) {
    world_ = _parent;
    node_ = gazebo_ros::Node::Get(_sdf);

    grid_topic_ = _sdf->Get<std::string>("grid_topic", grid_topic_).first;
    period_ = _sdf->Get<double>("period", period_).first;

    odom_listener_ = node_->create_subscription<nav_msgs::msg::Odometry>("/odom", 1, std::bind(&OccupancyGridPlugin::topic_callback, this, std::placeholders::_1));
    grid_publisher_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>(grid_topic_, 10);
    RCLCPP_INFO(node_->get_logger(), "Publish grid on [%s]", grid_topic_.c_str());

    map_resolution_ = 0.1;

    if(_sdf->HasElement("map_resolution"))
      map_resolution_ = _sdf->GetElement("map_resolution")->Get<double>();

    map_height_ = 0.3;

    if(_sdf->HasElement("map_z"))
      map_height_ = _sdf->GetElement("map_z")->Get<double>();

    init_robot_x_ = 0.0;
    
    if(_sdf->HasElement("init_robot_x"))
      init_robot_x_ = _sdf->GetElement("init_robot_x")->Get<double>();

    init_robot_y_ = 0.0;

    if(_sdf->HasElement("init_robot_y"))
      init_robot_y_ = _sdf->GetElement("init_robot_y")->Get<double>();

    map_size_x_ = 10.0;

    if(_sdf->HasElement("map_size_x"))
      map_size_x_ = _sdf->GetElement("map_size_x")->Get<double>();

    map_size_y_ = 10.0;

    if(_sdf->HasElement("map_size_y"))
      map_size_y_ = _sdf->GetElement("map_size_y")->Get<double>();

    // sdf::ElementPtr contactSensorSDF = _sdf->GetElement("contactSensor");

    // Listen to the update event (broadcast every simulation iteration)
    world_update_ = gazebo::event::Events::ConnectWorldUpdateBegin(
        std::bind(&OccupancyGridPlugin::OnWorldUpdate, this));
}

bool OccupancyGridPlugin::worldCellIntersection(const vector3d& cell_center,
                                                const double cell_length,
                                                gazebo::physics::RayShapePtr ray)

{
  //check for collisions with rays surrounding the cell
  //    ---
  //   | + |
  //    ---

  double dist;
  std::string entity_name;

  int cell_length_steps = 10;
  double side_length;

  //check for collisions with beams at increasing sizes to capture smaller
  //objects inside the cell
  for(int step=1; step<=cell_length_steps; step++)
  {
    side_length = cell_length / cell_length_steps * step;

    for(int i=-1; i<2; i+=2)
    {
        double start_x = cell_center.X() + i * side_length/2;
        double start_y = cell_center.Y() - i * side_length/2;

        for(int j=-1; j<2; j+=2)
        {   
            double end_x = cell_center.X() + j * side_length/2;
            double end_y = cell_center.Y() + j * side_length/2;

            ray->SetPoints(vector3d(start_x, start_y, cell_center.Z()),
                    vector3d(end_x, end_y, cell_center.Z()));
            ray->GetIntersection(dist, entity_name);
            
            if(!entity_name.empty() && !(entity_name.substr(0, 5)=="truck" && entity_name[5]==':'))
            {
                return true;
            }
        }
    }
}

  return false;
}

void OccupancyGridPlugin::cell2world(unsigned int cell_x, unsigned int cell_y,
                                       double map_size_x, double map_size_y,
                                       double map_resolution,
                                       double& world_x, double &world_y)
{
  world_x = cell_x * map_resolution - map_size_x/2 + map_resolution/2;
  world_y = cell_y * map_resolution - map_size_y/2 + map_resolution/2;
}

void OccupancyGridPlugin::world2cell(double world_x, double world_y,
                                       double map_size_x, double map_size_y,
                                       double map_resolution,
                                       unsigned int& cell_x, unsigned int& cell_y)
{
  cell_x = (world_x + map_size_x/2) / map_resolution;
  cell_y = (world_y + map_size_y/2) / map_resolution;
}

bool OccupancyGridPlugin::cell2index(int cell_x, int cell_y,
                                       unsigned int cell_size_x, unsigned int cell_size_y,
                                       unsigned int& map_index)
{
  if(cell_x >= 0 && cell_x < cell_size_x && cell_y >= 0 && cell_y < cell_size_y)
  {
    map_index = cell_y * cell_size_y + cell_x;
    return true;
  }
  else
  {
    //return false when outside map bounds
    return false;
  }
}

bool OccupancyGridPlugin::index2cell(int index, unsigned int cell_size_x,
                                       unsigned int cell_size_y,
                                       unsigned int& cell_x, unsigned int& cell_y)
{
  cell_y = index / cell_size_y;
  cell_x = index % cell_size_x;

  if(cell_x >= 0 && cell_x < cell_size_x && cell_y >= 0 && cell_y < cell_size_y)
    return true;
  else
  {
    //return false when outside map bounds
    return false;
  }
}


nav_msgs::msg::OccupancyGrid OccupancyGridPlugin::GetOccypancyGrid() {
    nav_msgs::msg::OccupancyGrid result;

    vector3d map_origin(0,0,map_height_);

    map_origin.X() = odometry_.pose.pose.position.x;
    map_origin.Y() = odometry_.pose.pose.position.y;
  
    unsigned int cells_size_x = map_size_x_ / map_resolution_;
    unsigned int cells_size_y = map_size_y_ / map_resolution_;

    result.header.frame_id = grid_frame_id_;
    result.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(world_->SimTime());

    result.info.map_load_time = rclcpp::Time(0);
    result.info.resolution = map_resolution_;
    result.info.width = cells_size_x;
    result.info.height = cells_size_y;

    result.info.origin.position.x = map_origin.X() - map_size_x_ / 2;
    result.info.origin.position.y = map_origin.Y() - map_size_y_ / 2;
    result.info.origin.position.z = map_origin.Z();

    result.info.origin.orientation.x = 0;
    result.info.origin.orientation.y = 0;
    result.info.origin.orientation.z = 0;
    result.info.origin.orientation.w = 1;

    result.data.resize(cells_size_x * cells_size_y);
    std::fill(result.data.begin(), result.data.end(), -1);

    gazebo::physics::PhysicsEnginePtr engine = world_->Physics();
    engine->InitForThread();

    gazebo::physics::RayShapePtr ray = boost::dynamic_pointer_cast<gazebo::physics::RayShape>(
        engine->CreateShape("ray", gazebo::physics::CollisionPtr()));

    //Starting wavefront expansion for mapping

    const double robot_x = init_robot_x_;
    const double robot_y = init_robot_y_;

    //initial cell
    unsigned int cell_x, cell_y, map_index;
    world2cell(robot_x, robot_y, map_size_x_, map_size_y_, map_resolution_,
             cell_x, cell_y);

    if(!cell2index(cell_x, cell_y, cells_size_x, cells_size_y, map_index))
    {
       return result;
    }

    std::vector<unsigned int> wavefront;
    wavefront.push_back(map_index);

    while(!wavefront.empty())
    {
        map_index = wavefront.at(0);
        wavefront.erase(wavefront.begin());

        index2cell(map_index, cells_size_x, cells_size_y, cell_x, cell_y);

        //0 means free cell
        result.data.at(map_index) = 0;

        //explore neighboring cells in a 8-connnected grid
        unsigned int child_index;
        double world_x, world_y;
        uint8_t child_val;

        //8-connected grid
        for (int i=-1;i<2;i++)
        {
            for(int j=-1;j<2;j++)
            {
                if(cell2index(cell_x + i, cell_y + j, cells_size_x, cells_size_y, child_index))
                {
                    child_val = result.data.at(child_index);

                    //if cell is unknown
                    if(child_val != 100 && child_val != 0 && child_val != 50)
                    {
                        cell2world(cell_x + i, cell_y + j, map_size_x_, map_size_y_, map_resolution_,
                                    world_x, world_y);

                        bool cell_occupied = worldCellIntersection(vector3d(world_x, world_y, map_height_), map_resolution_, ray);

                        if(cell_occupied)
                        {
                            result.data.at(child_index) = 100;
                        }
                        else
                        {
                            wavefront.push_back(child_index);
                            result.data.at(child_index) = 50;
                        }
                    }
                }
            }
        }
    }
    return result;
}

void OccupancyGridPlugin::OnWorldUpdate() {
    {
      std::lock_guard<std::mutex> guard(odometry_mutex_);
      odometry_ = last_odometry_;
    }

    const common::Time now = world_->SimTime();
    if (now - last_update_time_ < period_) {
        return;
    }

    grid_publisher_->publish(GetOccypancyGrid());
    last_update_time_ = now;
}

GZ_REGISTER_WORLD_PLUGIN(OccupancyGridPlugin)

}  // namespace gazebo