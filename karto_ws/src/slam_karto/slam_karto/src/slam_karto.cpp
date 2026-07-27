/*
 * slam_karto - Fixed for real-time map updates in RViz
 */

#define ROS_INFO RCUTILS_LOG_INFO
#define ROS_ERROR RCUTILS_LOG_ERROR
#define ROS_FATAL RCUTILS_LOG_FATAL
#define ROS_WARN RCUTILS_LOG_WARN
#define ROS_DEBUG RCUTILS_LOG_DEBUG

#include <string>
#include <map>
#include <vector>
#include <mutex>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/time_source.hpp" 

#include "nav_msgs/msg/map_meta_data.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav_msgs/srv/get_map.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "open_karto/Mapper.h"
#include "spa_solver.h"

#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

class SlamKarto
{
  public:
    SlamKarto(std::shared_ptr<rclcpp::Node> node);
    ~SlamKarto();

    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    bool mapCallback(const nav_msgs::srv::GetMap::Request::SharedPtr req,
                           nav_msgs::srv::GetMap::Response::SharedPtr res);

  private:
    bool getOdomPose(karto::Pose2& karto_pose, const rclcpp::Time& t);
    karto::LaserRangeFinder* getLaser(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    bool addScan(karto::LaserRangeFinder* laser,
                 const sensor_msgs::msg::LaserScan::SharedPtr scan,
                 karto::Pose2& karto_pose);
    bool updateMap();
    void publishLoop();
    void publishGraphVisualization();
    double getYaw(tf2::Transform& t);

    std::shared_ptr<rclcpp::Node> node;    
    std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf2_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf2B_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_filter_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr sst_;
    rclcpp::Publisher<nav_msgs::msg::MapMetaData>::SharedPtr sstm_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
    rclcpp::ServiceBase::SharedPtr ss_;
    rclcpp::TimerBase::SharedPtr timer_;

    nav_msgs::srv::GetMap::Response map_;

    std::string odom_frame_;
    std::string map_frame_;
    std::string base_frame_;
    int throttle_scans_;
    double resolution_;
    double map_update_interval_, transform_tolerance_;

    // THREADING FIXES: Added proper mutexes
    std::mutex map_mutex_;
    std::mutex map_to_odom_mutex_;
    std::mutex mapper_mutex_;  // CRITICAL: Protects all mapper operations
    std::mutex solver_mutex_;  // Protects solver access

    karto::Mapper* mapper_;
    karto::Dataset* dataset_;
    SpaSolver* solver_;
    std::map<std::string, karto::LaserRangeFinder*> lasers_;
    std::map<std::string, bool> lasers_inverted_;

    bool got_map_;
    int laser_count_;
    tf2::Transform map_to_odom_;
    unsigned marker_count_;
    bool inverted_laser_;
};

SlamKarto::SlamKarto(std::shared_ptr<rclcpp::Node> node_) :
        got_map_(false),
        laser_count_(0),
        marker_count_(0)
{
    node = node_;
    
    tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock(), tf2::durationFromSec(30.0));
    tf2_buffer_->setUsingDedicatedThread(true);
    tf2_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

    RCUTILS_LOG_INFO("Waiting for TF buffer to fill...");
    rclcpp::sleep_for(std::chrono::seconds(2));
    
    map_to_odom_.setIdentity();
  
    node->get_parameter_or("odom_frame", odom_frame_, std::string("odom"));  
    node->get_parameter_or("map_frame", map_frame_, std::string("map"));  
    node->get_parameter_or("base_frame", base_frame_, std::string("base_link"));
    node->get_parameter_or("throttle_scans", throttle_scans_, 1);
    node->get_parameter_or("resolution", resolution_, 0.05);
    node->get_parameter_or("delta", resolution_, 0.05);
    node->get_parameter_or("transform_tolerance", transform_tolerance_, 0.05);

    double tmp_sec;
    node->get_parameter_or("map_update_interval", tmp_sec, 5.0);
    map_update_interval_ = tmp_sec * 1e9; // Convert to nanoseconds
    
    double transform_publish_period;
    node->get_parameter_or("transform_publish_period", transform_publish_period, 0.05);

    tf2B_ = std::make_shared<tf2_ros::TransformBroadcaster>(node);
    
    // FIXED: Use transient_local for RViz compatibility
    sst_ = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "map", 
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
    
    sstm_ = node->create_publisher<nav_msgs::msg::MapMetaData>(
        "map_metadata", 
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
    
    marker_publisher_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
        "visualization_marker_array", 
        rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

    scan_filter_sub_ = node->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 
        rclcpp::SensorDataQoS(),
        std::bind(&SlamKarto::laserCallback, this, std::placeholders::_1));

    ss_ = node->create_service<nav_msgs::srv::GetMap>(
        "dynamic_map", 
        std::bind(&SlamKarto::mapCallback, this, std::placeholders::_1, std::placeholders::_2));

    auto loop_interval = std::chrono::milliseconds(int(transform_publish_period * 1000.0));
    timer_ = node->create_wall_timer(loop_interval, std::bind(&SlamKarto::publishLoop, this));

    mapper_ = new karto::Mapper();
    dataset_ = new karto::Dataset();

    // CRITICAL FIX: Force disable loop closure immediately after mapper creation
    ROS_INFO("FORCE DISABLING LOOP CLOSURE IN KARTO MAPPER");
    mapper_->setParamDoLoopClosing(false);
    
    // Set loop closure parameters to impossible values to ensure it's disabled
    mapper_->setParamLoopSearchMaximumDistance(0.0);
    mapper_->setParamLoopMatchMinimumChainSize(1000000);

    // Now set other parameters
    bool use_scan_matching;
    if(node->get_parameter("use_scan_matching", use_scan_matching)) {
        mapper_->setParamUseScanMatching(use_scan_matching);
        ROS_INFO("use_scan_matching: %s", use_scan_matching ? "true" : "false");
    }

    bool use_scan_barycenter;
    if(node->get_parameter("use_scan_barycenter", use_scan_barycenter)) {
        mapper_->setParamUseScanBarycenter(use_scan_barycenter);
        ROS_INFO("use_scan_barycenter: %s", use_scan_barycenter ? "true" : "false");
    }

    double minimum_travel_distance;
    if(node->get_parameter("minimum_travel_distance", minimum_travel_distance)) {
        mapper_->setParamMinimumTravelDistance(minimum_travel_distance);
        ROS_INFO("minimum_travel_distance: %.3f", minimum_travel_distance);
    }

    double minimum_travel_heading;
    if(node->get_parameter("minimum_travel_heading", minimum_travel_heading)) {
        mapper_->setParamMinimumTravelHeading(minimum_travel_heading);
        ROS_INFO("minimum_travel_heading: %.3f", minimum_travel_heading);
    }

    int scan_buffer_size;
    if(node->get_parameter("scan_buffer_size", scan_buffer_size)) {
        mapper_->setParamScanBufferSize(scan_buffer_size);
        ROS_INFO("scan_buffer_size: %d", scan_buffer_size);
    }

    double scan_buffer_maximum_scan_distance;
    if(node->get_parameter("scan_buffer_maximum_scan_distance", scan_buffer_maximum_scan_distance)) {
        mapper_->setParamScanBufferMaximumScanDistance(scan_buffer_maximum_scan_distance);
        ROS_INFO("scan_buffer_maximum_scan_distance: %.3f", scan_buffer_maximum_scan_distance);
    }

    double link_match_minimum_response_fine;
    if(node->get_parameter("link_match_minimum_response_fine", link_match_minimum_response_fine)) {
        mapper_->setParamLinkMatchMinimumResponseFine(link_match_minimum_response_fine);
        ROS_INFO("link_match_minimum_response_fine: %.3f", link_match_minimum_response_fine);
    }

    double link_scan_maximum_distance;
    if(node->get_parameter("link_scan_maximum_distance", link_scan_maximum_distance)) {
        mapper_->setParamLinkScanMaximumDistance(link_scan_maximum_distance);
        ROS_INFO("link_scan_maximum_distance: %.3f", link_scan_maximum_distance);
    }

    // CRITICAL: DO NOT SET LOOP CLOSURE PARAMETERS - THEY ARE FORCE DISABLED
    // Skip: loop_search_maximum_distance, loop_match_minimum_chain_size, etc.

    // Only set non-loop-closure parameters
    double correlation_search_space_dimension;
    if(node->get_parameter("correlation_search_space_dimension", correlation_search_space_dimension)) {
        mapper_->setParamCorrelationSearchSpaceDimension(correlation_search_space_dimension);
        ROS_INFO("correlation_search_space_dimension: %.3f", correlation_search_space_dimension);
    }

    double correlation_search_space_resolution;
    if(node->get_parameter("correlation_search_space_resolution", correlation_search_space_resolution)) {
        mapper_->setParamCorrelationSearchSpaceResolution(correlation_search_space_resolution);
        ROS_INFO("correlation_search_space_resolution: %.3f", correlation_search_space_resolution);
    }

    double correlation_search_space_smear_deviation;
    if(node->get_parameter("correlation_search_space_smear_deviation", correlation_search_space_smear_deviation)) {
        mapper_->setParamCorrelationSearchSpaceSmearDeviation(correlation_search_space_smear_deviation);
        ROS_INFO("correlation_search_space_smear_deviation: %.3f", correlation_search_space_smear_deviation);
    }

    // Skip loop search space parameters
    // Skip distance_variance_penalty, angle_variance_penalty if they cause issues

    solver_ = new SpaSolver();
    mapper_->SetScanSolver(solver_);
    
    ROS_INFO("SLAM Karto initialized successfully - LOOP CLOSURE DISABLED");
}

SlamKarto::~SlamKarto()
{
    if (solver_)
        delete solver_;
    if (mapper_)
        delete mapper_;
    if (dataset_)
        delete dataset_;
}

double
SlamKarto::getYaw(tf2::Transform& t)
{
  double yaw, pitch, roll;
  t.getBasis().getEulerYPR(yaw,pitch,roll);
  return yaw;
}

void 
SlamKarto::publishLoop()
{
    tf2::Transform map_to_odom_copy;
    
    // FIX: Quickly copy the transform with minimal locking
    {
        std::lock_guard<std::mutex> lock(map_to_odom_mutex_);
        map_to_odom_copy = map_to_odom_;
    }

    geometry_msgs::msg::TransformStamped tmp_tf_stamped;
    tmp_tf_stamped.header.frame_id = map_frame_;
    tmp_tf_stamped.child_frame_id = odom_frame_;
    tmp_tf_stamped.header.stamp = node->now() + rclcpp::Duration::from_seconds(transform_tolerance_);
    tmp_tf_stamped.transform = tf2::toMsg(map_to_odom_copy);
    tf2B_->sendTransform(tmp_tf_stamped);
}

karto::LaserRangeFinder* 
SlamKarto::getLaser(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
  if(lasers_.find(scan->header.frame_id) == lasers_.end())
  {
    tf2::Stamped<tf2::Transform> ident (tf2::Transform(tf2::Quaternion::getIdentity(),
                                       tf2::Vector3(0,0,0)), 
                                       tf2::TimePointZero,
                                       scan->header.frame_id);
    tf2::Stamped<tf2::Transform> laser_pose;

    try
    {
        geometry_msgs::msg::TransformStamped ident_msg = tf2::toMsg(ident);
        geometry_msgs::msg::TransformStamped laser_pose_msg;
        tf2_buffer_->transform(ident_msg, laser_pose_msg, base_frame_);
        tf2::fromMsg(laser_pose_msg, laser_pose);          
    }
    catch(const tf2::TransformException& e)
    {
      ROS_WARN("Failed to compute laser pose, aborting initialization (%s)", e.what());
      return NULL;
    }

    double yaw = getYaw(laser_pose);

    ROS_INFO("laser %s's pose wrt base: %.3f %.3f %.3f",
	     scan->header.frame_id.c_str(),
	     laser_pose.getOrigin().x(),
	     laser_pose.getOrigin().y(),
	     yaw);
    
    geometry_msgs::msg::Vector3Stamped up_in, up_out;
    up_in.header.stamp = scan->header.stamp;
    up_in.header.frame_id = base_frame_;
    up_in.vector.z = 1.0 + laser_pose.getOrigin().z();

    try
    {
        tf2_buffer_->transform(up_in, up_out, scan->header.frame_id);
        ROS_DEBUG("Z-Axis in sensor frame: %.3f", up_out.vector.z);
    }
    catch (const tf2::TransformException& e)
    {
      ROS_WARN("Unable to determine orientation of laser: %s", e.what());
      return NULL;
    }

    bool inverse = lasers_inverted_[scan->header.frame_id] = up_out.vector.z <= 0;
    if (inverse)
      ROS_INFO("laser is mounted upside-down");

    std::string name = scan->header.frame_id;
    karto::LaserRangeFinder* laser = 
      karto::LaserRangeFinder::CreateLaserRangeFinder(karto::LaserRangeFinder_Custom, karto::Name(name));
    laser->SetOffsetPose(karto::Pose2(laser_pose.getOrigin().x(),
				      laser_pose.getOrigin().y(),
				      yaw));
    laser->SetMinimumRange(scan->range_min);
    laser->SetMaximumRange(scan->range_max);
    laser->SetMinimumAngle(scan->angle_min);
    laser->SetMaximumAngle(scan->angle_max);
    laser->SetAngularResolution(scan->angle_increment);

    lasers_[scan->header.frame_id] = laser;
    dataset_->Add(laser);
  }

  return lasers_[scan->header.frame_id];
}

bool
SlamKarto::getOdomPose(karto::Pose2& karto_pose, const rclcpp::Time& t)
{
    tf2::Stamped<tf2::Transform> ident (tf2::Transform(tf2::Quaternion::getIdentity(),
                                        tf2::Vector3(0,0,0)), 
                                        tf2::TimePointZero,
                                        base_frame_);
    tf2::Stamped<tf2::Transform> odom_pose;

    try
    {
        geometry_msgs::msg::TransformStamped ident_msg = tf2::toMsg(ident);
        geometry_msgs::msg::TransformStamped odom_pose_msg;
        tf2_buffer_->transform(ident_msg, odom_pose_msg, odom_frame_);
        tf2::fromMsg(odom_pose_msg, odom_pose);   
    }
    catch(const tf2::TransformException& e)
    {
        ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
        return false;
    }
    double yaw = getYaw(odom_pose);

    karto_pose = karto::Pose2(odom_pose.getOrigin().x(),
                   odom_pose.getOrigin().y(),
                   yaw);
    return true;
}

void
SlamKarto::publishGraphVisualization()
{
    std::vector<float> graph;
    
    // FIX: Protect solver access with mutex
    {
        std::lock_guard<std::mutex> solver_lock(solver_mutex_);
        solver_->getGraph(graph);
    }

    visualization_msgs::msg::MarkerArray marray;

    visualization_msgs::msg::Marker m;
    m.header.frame_id = "map";
    m.header.stamp = node->now();
    m.id = 0;
    m.ns = "karto";
    m.type = 2;
    m.pose.position.x = 0.0;
    m.pose.position.y = 0.0;
    m.pose.position.z = 0.0;
    m.scale.x = 0.1;
    m.scale.y = 0.1;
    m.scale.z = 0.1;
    m.color.r = 1.0;
    m.color.g = 0;
    m.color.b = 0.0;
    m.color.a = 1.0;
    m.lifetime = rclcpp::Duration(0,0);

    visualization_msgs::msg::Marker edge;
    edge.header.frame_id = "map";
    edge.header.stamp = node->now();
    edge.action = 0;
    edge.ns = "karto";
    edge.id = 0;
    edge.type = 4;
    edge.scale.x = 0.1;
    edge.scale.y = 0.1;
    edge.scale.z = 0.1;
    edge.color.a = 1.0;
    edge.color.r = 0.0;
    edge.color.g = 0.0;
    edge.color.b = 1.0;
  
    m.action = 0;
    uint id = 0;
    for (uint i=0; i<graph.size()/2; i++) 
    {
        m.id = id;
        m.pose.position.x = graph[2*i];
        m.pose.position.y = graph[2*i+1];
        marray.markers.push_back(visualization_msgs::msg::Marker(m));
        id++;

        if(i>0)
        {
            edge.points.clear();

            geometry_msgs::msg::Point p;
            p.x = graph[2*(i-1)];
            p.y = graph[2*(i-1)+1];
            edge.points.push_back(p);
            p.x = graph[2*i];
            p.y = graph[2*i+1];
            edge.points.push_back(p);
            edge.id = id;

            marray.markers.push_back(visualization_msgs::msg::Marker(edge));
            id++;
        }
    }

    m.action = 2;
    for (; id < marker_count_; id++) 
    {
        m.id = id;
        marray.markers.push_back(visualization_msgs::msg::Marker(m));
    }

    marker_count_ = marray.markers.size();
    marker_publisher_->publish(marray);
}

void
SlamKarto::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
    laser_count_++;
    if ((laser_count_ % throttle_scans_) != 0)
        return;

    static rclcpp::Time last_map_update(0,0,RCL_ROS_TIME);

    karto::LaserRangeFinder* laser = getLaser(scan);

    if(!laser)
    {
        ROS_WARN("Failed to create laser device for %s; discarding scan",
             scan->header.frame_id.c_str());
        return;
    }

    karto::Pose2 odom_pose;
    if(addScan(laser, scan, odom_pose))
    {
        ROS_DEBUG("added scan at pose: %.3f %.3f %.3f", 
                  odom_pose.GetX(),
                  odom_pose.GetY(),
                  odom_pose.GetHeading());

        publishGraphVisualization();

        if(!got_map_ || 
           (node->now() - last_map_update).nanoseconds() > map_update_interval_)
        {
            if(updateMap())
            {
                last_map_update = node->now();
                got_map_ = true;
                ROS_DEBUG("Updated the map");
            }
        }
    }
}

bool
SlamKarto::updateMap()
{
    std::unique_lock<std::mutex> map_lock(map_mutex_);
    std::lock_guard<std::mutex> mapper_lock(mapper_mutex_);  // FIX: Protect mapper access

    karto::OccupancyGrid* occ_grid = 
            karto::OccupancyGrid::CreateFromScans(mapper_->GetAllProcessedScans(), resolution_);

    if(!occ_grid)
        return false;

    if(!got_map_) {
        map_.map.info.resolution = resolution_;
        map_.map.info.origin.position.x = 0.0;
        map_.map.info.origin.position.y = 0.0;
        map_.map.info.origin.position.z = 0.0;
        map_.map.info.origin.orientation.x = 0.0;
        map_.map.info.origin.orientation.y = 0.0;
        map_.map.info.origin.orientation.z = 0.0;
        map_.map.info.origin.orientation.w = 1.0;
    } 

    kt_int32s width = occ_grid->GetWidth();
    kt_int32s height = occ_grid->GetHeight();
    karto::Vector2<kt_double> offset = occ_grid->GetCoordinateConverter()->GetOffset();

    if(map_.map.info.width != (unsigned int) width || 
       map_.map.info.height != (unsigned int) height ||
       map_.map.info.origin.position.x != offset.GetX() ||
       map_.map.info.origin.position.y != offset.GetY())
    {
        map_.map.info.origin.position.x = offset.GetX();
        map_.map.info.origin.position.y = offset.GetY();
        map_.map.info.width = width;
        map_.map.info.height = height;
        map_.map.data.resize(map_.map.info.width * map_.map.info.height);
    }

    for (kt_int32s y=0; y<height; y++)
    {
        for (kt_int32s x=0; x<width; x++) 
        {
            kt_int8u value = occ_grid->GetValue(karto::Vector2<kt_int32s>(x, y));

            switch (value)
            {
                case karto::GridStates_Unknown:
                    map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = -1;
                    break;
                case karto::GridStates_Occupied:
                    map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = 100;
                    break;
                case karto::GridStates_Free:
                    map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = 0;
                    break;
                default:
                    ROS_WARN("Encountered unknown cell value at %d, %d", x, y);
                    break;
            }
        }
    }
  
    map_.map.header.stamp = node->now();
    map_.map.header.frame_id = map_frame_;

    sst_->publish(map_.map);
    sstm_->publish(map_.map.info);

    delete occ_grid;

    return true;
}

bool
SlamKarto::addScan(karto::LaserRangeFinder* laser,
                   const sensor_msgs::msg::LaserScan::SharedPtr scan, 
                   karto::Pose2& karto_pose)
{
    if(!getOdomPose(karto_pose, scan->header.stamp))
        return false;

    std::vector<kt_double> readings;

    if (lasers_inverted_[scan->header.frame_id]) 
    {
        for(std::vector<float>::const_reverse_iterator it = scan->ranges.rbegin();
            it != scan->ranges.rend();
            ++it)
        {
            readings.push_back(*it);
        }
    } 
    else
    {
        for(std::vector<float>::const_iterator it = scan->ranges.begin();
            it != scan->ranges.end();
            ++it)
        {
            readings.push_back(*it);
        }
    }

    karto::LocalizedRangeScan* range_scan = new karto::LocalizedRangeScan(laser->GetName(), readings);
    range_scan->SetOdometricPose(karto_pose);
    range_scan->SetCorrectedPose(karto_pose);

    bool processed;
    
    // FIX: PROTECT THE ENTIRE MAPPER PROCESS WITH A MUTEX
    {
        std::lock_guard<std::mutex> mapper_lock(mapper_mutex_);
        
        if((processed = mapper_->Process(range_scan)))
        {
            karto::Pose2 corrected_pose = range_scan->GetCorrectedPose();

            tf2::Quaternion q;
            q.setRPY(0.0, 0.0, corrected_pose.GetHeading());
            tf2::Transform tmp_tf(q, tf2::Vector3(corrected_pose.GetX(),
                                                  corrected_pose.GetY(),
                                                  0.0));
            tf2::Stamped<tf2::Transform> input (tmp_tf.inverse(), 
                                        tf2::TimePointZero,
                                        base_frame_);
            tf2::Stamped<tf2::Transform> odom_to_map;

            try
            {
                geometry_msgs::msg::TransformStamped input_msg = tf2::toMsg(input);
                geometry_msgs::msg::TransformStamped odom_to_map_msg;
                tf2_buffer_->transform(input_msg, odom_to_map_msg, odom_frame_);
                tf2::fromMsg(odom_to_map_msg, odom_to_map);   
            }
            catch(const tf2::TransformException& e)
            {
                ROS_ERROR("Transform from base_link to odom failed: %s", e.what());
                odom_to_map.setIdentity();
            }
            
            {
                std::lock_guard<std::mutex> lock(map_to_odom_mutex_);
                map_to_odom_ = tf2::Transform(tf2::Quaternion(odom_to_map.getRotation()),
                                              tf2::Vector3(odom_to_map.getOrigin())).inverse();
            }

            dataset_->Add(range_scan);
        }
        else
        {
            delete range_scan;
        }
    }

    return processed;
}

bool 
SlamKarto::mapCallback(const nav_msgs::srv::GetMap::Request::SharedPtr req,
                             nav_msgs::srv::GetMap::Response::SharedPtr res)
{
    (void)req;
    std::unique_lock<std::mutex> lock(map_mutex_);
    if(got_map_ && map_.map.info.width && map_.map.info.height)
    {
        *res = map_;
        return true;
    }
    else
        return false;
}

int
main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("slam_karto");
    std::shared_ptr<SlamKarto> node_ptr;
    node_ptr.reset(new SlamKarto(node));
    
    rclcpp::spin(node);
    node_ptr.reset();
    
    return 0;
}
