/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Robert Bosch LLC.
 *  Copyright (c) 2015-2016, Jiri Horner.
 *  Copyright (c) 2021, Carlos Alvarez, Juan Galvis.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Jiri Horner nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

#include <explore/explore.h>

#include <thread>
#include <std_srvs/srv/empty.hpp>

inline static bool same_point(const geometry_msgs::msg::Point& one,
                              const geometry_msgs::msg::Point& two)
{
  double dx = one.x - two.x;
  double dy = one.y - two.y;
  double dist = sqrt(dx * dx + dy * dy);
  return dist < 0.01;
}

namespace explore
{
Explore::Explore()
  : Node("explore_node")
  , logger_(this->get_logger())
  , tf_buffer_(this->get_clock())
  , tf_listener_(tf_buffer_)
  , costmap_client_(*this, &tf_buffer_)
  , prev_distance_(0)
  , last_progress_(this->now())
  , last_markers_count_(0)
{
  double timeout;
  double min_frontier_size;
  this->declare_parameter<float>("planner_frequency", 1.0);
  this->declare_parameter<float>("progress_timeout", 30.0);
  this->declare_parameter<bool>("visualize", false);
  this->declare_parameter<float>("potential_scale", 1e-3);
  this->declare_parameter<float>("orientation_scale", 0.0);
  this->declare_parameter<float>("gain_scale", 1.0);
  this->declare_parameter<float>("min_frontier_size", 0.5);
  this->declare_parameter<bool>("return_to_init", false);
  this->declare_parameter<bool>("revisit_enabled", false);
  this->declare_parameter<int>("revisit_after_n_goals", 5);

  this->get_parameter("planner_frequency", planner_frequency_);
  this->get_parameter("progress_timeout", timeout);
  this->get_parameter("visualize", visualize_);
  this->get_parameter("potential_scale", potential_scale_);
  this->get_parameter("orientation_scale", orientation_scale_);
  this->get_parameter("gain_scale", gain_scale_);
  this->get_parameter("min_frontier_size", min_frontier_size);
  this->get_parameter("return_to_init", return_to_init_);
  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("revisit_enabled", revisit_enabled_);
  this->get_parameter("revisit_after_n_goals", revisit_after_n_goals_);

  progress_timeout_ = timeout;
  move_base_client_ =
      rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
          this, ACTION_NAME);

  search_ = frontier_exploration::FrontierSearch(costmap_client_.getCostmap(),
                                                 potential_scale_, gain_scale_,
                                                 min_frontier_size, logger_);

  if (visualize_) {
    marker_array_publisher_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>("explore/"
                                                                     "frontier"
                                                                     "s",
                                                                     10);
  }

  // Subscription to resume or stop exploration
  resume_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
      "explore/resume", 10,
      std::bind(&Explore::resumeCallback, this, std::placeholders::_1));

  RCLCPP_INFO(logger_, "Waiting to connect to move_base nav2 server");
  move_base_client_->wait_for_action_server();
  RCLCPP_INFO(logger_, "Connected to move_base nav2 server");

  if (return_to_init_ || revisit_enabled_) {
    RCLCPP_INFO(logger_, "Getting initial pose of the robot");
    geometry_msgs::msg::TransformStamped transformStamped;
    std::string map_frame = costmap_client_.getGlobalFrameID();
    try {
      transformStamped = tf_buffer_.lookupTransform(
          map_frame, robot_base_frame_, tf2::TimePointZero);
      initial_pose_.position.x = transformStamped.transform.translation.x;
      initial_pose_.position.y = transformStamped.transform.translation.y;
      initial_pose_.orientation = transformStamped.transform.rotation;
    } catch (tf2::TransformException& ex) {
      RCLCPP_ERROR(logger_, "Couldn't find transform from %s to %s: %s",
                   map_frame.c_str(), robot_base_frame_.c_str(), ex.what());
      return_to_init_ = false;
      revisit_enabled_ = false;
    }
  }

  exploring_timer_ = this->create_wall_timer(
      std::chrono::milliseconds((uint16_t)(1000.0 / planner_frequency_)),
      [this]() { makePlan(); });
  RCLCPP_INFO(logger_, "Exploration timer started with frequency %.2f Hz", planner_frequency_);
  // Start exploration right away
  makePlan();
}

Explore::~Explore()
{
  stop();
}

void Explore::resumeCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data) {
    resume();
  } else {
    stop();
  }
}

void Explore::visualizeFrontiers(
    const std::vector<frontier_exploration::Frontier>& frontiers)
{
  std_msgs::msg::ColorRGBA blue;
  blue.r = 0;
  blue.g = 0;
  blue.b = 1.0;
  blue.a = 1.0;
  std_msgs::msg::ColorRGBA red;
  red.r = 1.0;
  red.g = 0;
  red.b = 0;
  red.a = 1.0;
  std_msgs::msg::ColorRGBA green;
  green.r = 0;
  green.g = 1.0;
  green.b = 0;
  green.a = 1.0;

  RCLCPP_DEBUG(logger_, "visualising %lu frontiers", frontiers.size());
  visualization_msgs::msg::MarkerArray markers_msg;
  std::vector<visualization_msgs::msg::Marker>& markers = markers_msg.markers;
  visualization_msgs::msg::Marker m;

  m.header.frame_id = costmap_client_.getGlobalFrameID();
  m.header.stamp = this->now();
  m.ns = "frontiers";
  m.scale.x = 1.0;
  m.scale.y = 1.0;
  m.scale.z = 1.0;
  m.color.r = 0;
  m.color.g = 0;
  m.color.b = 255;
  m.color.a = 255;
  // lives forever
#ifdef ELOQUENT
  m.lifetime = rclcpp::Duration(0);  // deprecated in galactic warning
#elif DASHING
  m.lifetime = rclcpp::Duration(0);  // deprecated in galactic warning
#else
  m.lifetime = rclcpp::Duration::from_seconds(0);  // foxy onwards
#endif
  // m.lifetime = rclcpp::Duration::from_nanoseconds(0); // suggested in
  // galactic
  m.frame_locked = true;

  // weighted frontiers are always sorted
  double min_cost = frontiers.empty() ? 0. : frontiers.front().cost;

  m.action = visualization_msgs::msg::Marker::ADD;
  size_t id = 0;
  for (auto& frontier : frontiers) {
    m.type = visualization_msgs::msg::Marker::POINTS;
    m.id = int(id);
    // m.pose.position = {}; // compile warning
    m.scale.x = 0.1;
    m.scale.y = 0.1;
    m.scale.z = 0.1;
    m.points = frontier.points;
    if (goalOnBlacklist(frontier.centroid)) {
      m.color = red;
    } else {
      m.color = blue;
    }
    markers.push_back(m);
    ++id;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.id = int(id);
    m.pose.position = frontier.initial;
    // scale frontier according to its cost (costier frontiers will be smaller)
    double scale = std::min(std::abs(min_cost * 0.4 / frontier.cost), 0.5);
    m.scale.x = scale;
    m.scale.y = scale;
    m.scale.z = scale;
    m.points = {};
    m.color = green;
    markers.push_back(m);
    ++id;
  }
  size_t current_markers_count = markers.size();

  // delete previous markers, which are now unused
  m.action = visualization_msgs::msg::Marker::DELETE;
  for (; id < last_markers_count_; ++id) {
    m.id = int(id);
    markers.push_back(m);
  }

  last_markers_count_ = current_markers_count;
  marker_array_publisher_->publish(markers_msg);
}

void Explore::makePlan()
{
  RCLCPP_DEBUG(logger_, "makePlan() called");
  
  // Check if transform is available before getting robot pose
  std::string global_frame = costmap_client_.getGlobalFrameID();
  std::string tf_error;
  if (!tf_buffer_.canTransform(global_frame, robot_base_frame_,
                               tf2::TimePointZero, tf2::durationFromSec(0.1),
                               &tf_error)) {
    RCLCPP_WARN_THROTTLE(logger_, *this->get_clock(), 5000,
                         "Transform from %s to %s not available yet. "
                         "Waiting for AMCL to initialize. Error: %s",
                         robot_base_frame_.c_str(), global_frame.c_str(),
                         tf_error.c_str());
    return;  // Retry on next timer tick
  }

  // find frontiers
  auto pose = costmap_client_.getRobotPose();
  
  // Check if pose is valid (orientation should be non-zero for valid quaternion)
  // A default-constructed pose has orientation (0,0,0,0) which is invalid
  if (pose.orientation.w == 0.0 && pose.orientation.x == 0.0 &&
      pose.orientation.y == 0.0 && pose.orientation.z == 0.0) {
    RCLCPP_WARN_THROTTLE(logger_, *this->get_clock(), 5000,
                         "Robot pose is invalid (empty pose returned). "
                         "Waiting for valid transform from AMCL.");
    return;  // Retry on next timer tick
  }
  
  // Track pose stability and handle localization failures
  static geometry_msgs::msg::Point last_pose = pose.position;
  static int pose_jump_count = 0;
  static rclcpp::Time last_pose_jump_time = this->now();
  static bool localization_recovery_mode = false;
  static rclcpp::Time recovery_start_time = this->now();
  static rclcpp::Time last_reinit_attempt = this->now();
  static std::shared_ptr<rclcpp::Client<std_srvs::srv::Empty>> reinit_client = nullptr;
  static double last_jump_magnitude = 0.0;
  static double max_jump_magnitude = 0.0;
  static int reinit_attempt_count = 0;
  
  // Create service client if it doesn't exist
  if (!reinit_client) {
    reinit_client = this->create_client<std_srvs::srv::Empty>("reinitialize_global_localization");
  }
  
  double pose_delta = sqrt(pow(pose.position.x - last_pose.x, 2) + 
                           pow(pose.position.y - last_pose.y, 2));
  
  // Check for pose jumps (localization failures)
  if (pose_delta > 0.5) {  // More than 0.5m jump
    pose_jump_count++;
    last_pose_jump_time = this->now();
    last_jump_magnitude = pose_delta;
    if (pose_delta > max_jump_magnitude) {
      max_jump_magnitude = pose_delta;
    }
    
    RCLCPP_WARN(logger_, "Large pose jump detected: %.2f m (from %.2f,%.2f to %.2f,%.2f). "
                "This may indicate AMCL instability. Jump count: %d",
                pose_delta, last_pose.x, last_pose.y, pose.position.x, pose.position.y, pose_jump_count);
    
    // Enter recovery mode if too many jumps in short time (increased threshold to 5)
    // This prevents false triggers from temporary dynamic obstacles (people walking by)
    if (pose_jump_count >= 5 && !localization_recovery_mode) {
      localization_recovery_mode = true;
      recovery_start_time = this->now();
      last_reinit_attempt = this->now();
      reinit_attempt_count = 0;
      max_jump_magnitude = pose_delta;  // Reset max for this recovery session
      RCLCPP_ERROR(logger_, 
                   "Localization appears lost! Entering recovery mode. "
                   "Pausing exploration to allow AMCL to recover. "
                   "This may be caused by dynamic obstacles (people).");
      
      // Cancel current navigation goal
      if (goal_in_progress_) {
        RCLCPP_INFO(logger_, "Canceling current navigation goal due to localization failure");
        goal_in_progress_ = false;
      }
      
      // DON'T reinitialize immediately - wait to see if AMCL stabilizes on its own
      // Reinitialization often makes things worse when dynamic obstacles are present
      RCLCPP_INFO(logger_, "Waiting to see if AMCL stabilizes before attempting reinitialization...");
    }
  }
  
  // Check if we should exit recovery mode or retry reinitialization
  if (localization_recovery_mode) {
    rclcpp::Duration time_in_recovery = this->now() - recovery_start_time;
    rclcpp::Duration time_since_last_jump = this->now() - last_pose_jump_time;
    rclcpp::Duration time_since_last_reinit = this->now() - last_reinit_attempt;
    
    // Smart reinitialization strategy:
    // - Only reinitialize if jumps are getting SMALLER (AMCL might be recovering)
    // - Don't reinitialize if jumps are getting LARGER (would make it worse)
    // - Wait at least 10 seconds before first reinitialization attempt
    // - Only reinitialize once every 15 seconds max
    bool should_reinit = false;
    if (time_since_last_reinit.seconds() >= 15.0 && time_in_recovery.seconds() >= 10.0) {
      // Check if jumps are getting smaller (sign of recovery)
      if (last_jump_magnitude < max_jump_magnitude * 0.7) {
        // Jumps are getting smaller - AMCL might be recovering, try reinitialization
        should_reinit = true;
        RCLCPP_INFO(logger_, 
                    "Jump magnitude decreasing (%.2f m < %.2f m). "
                    "Attempting AMCL reinitialization to help recovery.",
                    last_jump_magnitude, max_jump_magnitude);
      } else if (reinit_attempt_count == 0 && time_in_recovery.seconds() >= 15.0) {
        // First attempt after waiting - try once even if jumps aren't decreasing
        should_reinit = true;
        RCLCPP_INFO(logger_, 
                    "First reinitialization attempt after %.1f seconds in recovery.",
                    time_in_recovery.seconds());
      } else {
        // Jumps are still large or increasing - don't reinitialize (would make it worse)
        RCLCPP_DEBUG_THROTTLE(logger_, *this->get_clock(), 10000,
                              "Jumps still large (%.2f m). Skipping reinitialization to avoid making it worse.",
                              last_jump_magnitude);
      }
    }
    
    if (should_reinit) {
      if (reinit_client->wait_for_service(std::chrono::seconds(1))) {
        auto request = std::make_shared<std_srvs::srv::Empty::Request>();
        auto result = reinit_client->async_send_request(request);
        last_reinit_attempt = this->now();
        reinit_attempt_count++;
        RCLCPP_INFO(logger_, 
                    "Triggered AMCL global re-initialization (attempt %d, %.1f seconds in recovery)",
                    reinit_attempt_count, time_in_recovery.seconds());
      } else {
        RCLCPP_WARN_THROTTLE(logger_, *this->get_clock(), 10000,
                            "AMCL reinitialize service not available");
      }
    }
    
    // Exit recovery mode if:
    // 1. No jumps for 15 seconds AND we've been in recovery for at least 10 seconds (pose stabilized)
    // OR
    // 2. We've been in recovery for more than 60 seconds (give up and resume anyway)
    //    Increased timeout to give AMCL more time to recover from dynamic obstacles
    bool should_exit = false;
    if (time_since_last_jump.seconds() > 15.0 && time_in_recovery.seconds() > 10.0) {
      should_exit = true;
      RCLCPP_INFO(logger_, 
                  "Localization appears stable (no jumps for %.1f seconds). "
                  "Exiting recovery mode and resuming exploration.",
                  time_since_last_jump.seconds());
    } else if (time_in_recovery.seconds() > 60.0) {
      should_exit = true;
      RCLCPP_WARN(logger_, 
                  "Recovery mode timeout (%.1f seconds). Resuming exploration despite ongoing pose jumps. "
                  "AMCL may need manual intervention or dynamic obstacles may need to clear.",
                  time_in_recovery.seconds());
    }
    
    if (should_exit) {
      localization_recovery_mode = false;
      pose_jump_count = 0;  // Reset counter
      max_jump_magnitude = 0.0;
      reinit_attempt_count = 0;
    } else {
      // Still in recovery - don't plan new goals
      RCLCPP_INFO_THROTTLE(logger_, *this->get_clock(), 5000,
                           "In localization recovery mode. Waiting for AMCL to stabilize... "
                           "(%.1f seconds since last jump, %.1f seconds in recovery, "
                           "jump count: %d, max jump: %.2f m, last jump: %.2f m)",
                           time_since_last_jump.seconds(), time_in_recovery.seconds(), 
                           pose_jump_count, max_jump_magnitude, last_jump_magnitude);
      return;  // Skip planning while in recovery
    }
  }
  
  last_pose = pose.position;

  // Check if costmap has unknown space before searching for frontiers
  auto* costmap = costmap_client_.getCostmap();
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));
  const unsigned char* map_data = costmap->getCharMap();
  size_t map_size = costmap->getSizeInCellsX() * costmap->getSizeInCellsY();
  
  // Count unknown cells (NO_INFORMATION = 255)
  size_t unknown_count = 0;
  size_t free_count = 0;
  size_t occupied_count = 0;
  for (size_t i = 0; i < map_size; ++i) {
    if (map_data[i] == 255) {  // NO_INFORMATION
      unknown_count++;
    } else if (map_data[i] == 0) {  // FREE_SPACE
      free_count++;
    } else if (map_data[i] >= 253) {  // LETHAL_OBSTACLE or INSCRIBED
      occupied_count++;
    }
  }
  
  // Log costmap stats (not throttled) when the map content changes
  static size_t last_unknown_count = 0;
  static size_t last_free_count = 0;
  bool costmap_changed = (unknown_count != last_unknown_count ||
                          free_count != last_free_count);
  last_unknown_count = unknown_count;
  last_free_count = free_count;

  if (costmap_changed) {
    RCLCPP_INFO(logger_,
                "Costmap stats - Unknown: %zu (%.1f%%), Free: %zu (%.1f%%), "
                "Occupied: %zu (%.1f%%), Total: %zu",
                unknown_count, 100.0 * unknown_count / map_size,
                free_count,   100.0 * free_count   / map_size,
                occupied_count, 100.0 * occupied_count / map_size,
                map_size);
  }
  
  if (unknown_count == 0) {
    RCLCPP_WARN_THROTTLE(logger_, *this->get_clock(), 5000,
                         "Costmap has no unknown space (NO_INFORMATION cells). "
                         "This might indicate the costmap is not tracking unknown space. "
                         "Check that global_costmap has 'track_unknown_space: true' and "
                         "static_layer uses 'use_maximum: false' or 'combination_method: Overwrite'.");
    // Don't stop, just wait - the map might update
    return;
  }

  // Check robot position in costmap coordinates for debugging
  unsigned int robot_mx, robot_my;
  bool robot_in_bounds = costmap->worldToMap(pose.position.x, pose.position.y, robot_mx, robot_my);
  if (!robot_in_bounds) {
    RCLCPP_WARN_THROTTLE(logger_, *this->get_clock(), 5000,
                         "Robot pose (%.2f, %.2f) is outside costmap bounds. "
                         "Cannot search for frontiers.",
                         pose.position.x, pose.position.y);
    return;
  }
  
  // Check what's around the robot position
  unsigned int robot_idx = costmap->getIndex(robot_mx, robot_my);
  unsigned char robot_cell = map_data[robot_idx];
  RCLCPP_INFO(logger_, "Robot at costmap cell (%u, %u), value: %u (0=free, 255=unknown, 254=lethal), pose: (%.2f, %.2f)",
               robot_mx, robot_my, robot_cell, pose.position.x, pose.position.y);
  
  // Check a small area around robot for unknown cells
  int check_radius = 10;  // cells
  size_t nearby_unknown = 0;
  size_t nearby_free = 0;
  for (int dy = -check_radius; dy <= check_radius; ++dy) {
    for (int dx = -check_radius; dx <= check_radius; ++dx) {
      int check_x = (int)robot_mx + dx;
      int check_y = (int)robot_my + dy;
      if (check_x >= 0 && check_x < (int)costmap->getSizeInCellsX() &&
          check_y >= 0 && check_y < (int)costmap->getSizeInCellsY()) {
        unsigned int check_idx = costmap->getIndex(check_x, check_y);
        if (map_data[check_idx] == 255) nearby_unknown++;
        else if (map_data[check_idx] == 0) nearby_free++;
      }
    }
  }
  RCLCPP_INFO(logger_, "Nearby cells (radius %d): %zu unknown, %zu free", 
              check_radius, nearby_unknown, nearby_free);

  // Ensure map has minimum data before searching for frontiers
  // This prevents premature "no frontiers" when map is still initializing
  size_t min_map_cells_for_exploration = 1000;  // Require at least 1000 cells of data
  if (map_size < min_map_cells_for_exploration) {
    RCLCPP_DEBUG_THROTTLE(logger_, *this->get_clock(), 5000,
                         "Map too small (%zu cells), waiting for more data before exploring. "
                         "Need at least %zu cells.",
                         map_size, min_map_cells_for_exploration);
    return;  // Wait for more map data
  }

  // get frontiers sorted according to cost
  auto frontiers = search_.searchFrom(pose.position);
  RCLCPP_INFO(logger_, "Found %lu frontiers", frontiers.size());
  for (size_t i = 0; i < frontiers.size() && i < 5; ++i) {  // Log first 5 frontiers
    RCLCPP_INFO(logger_, "  Frontier %zu: cost=%.2f, distance=%.2f, size=%u", 
                i, frontiers[i].cost, frontiers[i].min_distance, frontiers[i].size);
  }

  if (frontiers.empty()) {
    // Check if there are unknown cells adjacent to free space
    size_t unknown_with_free_neighbor = 0;
    for (size_t i = 0; i < map_size; ++i) {
      if (map_data[i] == 255) {  // NO_INFORMATION
        // Check 4-connected neighbors
        unsigned int x = i % costmap->getSizeInCellsX();
        unsigned int y = i / costmap->getSizeInCellsX();
        bool has_free_neighbor = false;
        // Check left
        if (x > 0 && map_data[i - 1] == 0) has_free_neighbor = true;
        // Check right
        if (x < costmap->getSizeInCellsX() - 1 && map_data[i + 1] == 0) has_free_neighbor = true;
        // Check bottom
        if (y > 0 && map_data[i - costmap->getSizeInCellsX()] == 0) has_free_neighbor = true;
        // Check top
        if (y < costmap->getSizeInCellsY() - 1 && 
            map_data[i + costmap->getSizeInCellsX()] == 0) has_free_neighbor = true;
        if (has_free_neighbor) unknown_with_free_neighbor++;
      }
    }
    
    RCLCPP_WARN(logger_,
                "No frontiers found! Stats: %zu unknown cells (%.1f%%), "
                "%zu unknown cells with free neighbors, "
                "%zu free cells (%.1f%%). Robot at (%.2f, %.2f), cell (%u,%u), value: %u. "
                "Nearby: %zu unknown, %zu free",
                unknown_count, 100.0*unknown_count/map_size,
                unknown_with_free_neighbor, 
                free_count, 100.0*free_count/map_size,
                pose.position.x, pose.position.y, robot_mx, robot_my, robot_cell,
                nearby_unknown, nearby_free);
    // Don't stop immediately - wait a bit and retry
    return;  // Retry on next timer tick instead of stopping
  }

  // publish frontiers as visualization markers
  if (visualize_) {
    visualizeFrontiers(frontiers);
  }

  // Filter out frontiers that are too close to the robot (likely at robot's location)
  // This prevents the robot from trying to "explore" frontiers that are essentially at its current position
  // Use adaptive threshold: stricter initially, but allow closer frontiers if robot is stuck
  static int too_close_count = 0;
  static rclcpp::Time first_too_close_time = this->now();
  static bool stuck_timer_started = false;
  
  const double min_frontier_distance_strict = 0.15;  // Strict minimum (prevents immediate loops, allows 0.16m+ frontiers)
  const double min_frontier_distance_relaxed = 0.08;  // Relaxed minimum (allows progress when stuck)
  
  // If we've been stuck for more than 5 seconds, use relaxed threshold
  rclcpp::Duration stuck_duration(0, 0);
  if (stuck_timer_started) {
    stuck_duration = this->now() - first_too_close_time;
  }
  double min_frontier_distance = (stuck_duration.seconds() > 5.0) ? 
                                  min_frontier_distance_relaxed : 
                                  min_frontier_distance_strict;
  
  std::vector<frontier_exploration::Frontier> valid_frontiers;
  for (const auto& f : frontiers) {
    if (f.min_distance >= min_frontier_distance) {
      valid_frontiers.push_back(f);
      RCLCPP_DEBUG(logger_, 
                   "Accepting frontier at (%.2f, %.2f) - distance %.2f m >= %.2f m threshold",
                   f.centroid.x, f.centroid.y, f.min_distance, min_frontier_distance);
      // Reset stuck counter if we find a valid frontier
      too_close_count = 0;
      stuck_timer_started = false;
      first_too_close_time = this->now();
    } else {
      RCLCPP_INFO(logger_, 
                   "Filtering out frontier at (%.2f, %.2f) - too close (%.2f m < %.2f m threshold)",
                   f.centroid.x, f.centroid.y, f.min_distance, min_frontier_distance);
    }
  }
  
  if (!valid_frontiers.empty()) {
    RCLCPP_INFO(logger_, "After distance filtering: %zu valid frontiers (from %zu total)",
                valid_frontiers.size(), frontiers.size());
  }
  
  if (valid_frontiers.empty() && !frontiers.empty()) {
    too_close_count++;
    if (!stuck_timer_started) {
      first_too_close_time = this->now();
      stuck_timer_started = true;
    }
    
    RCLCPP_WARN_THROTTLE(logger_, *this->get_clock(), 2000,
                "All %zu frontiers are too close to robot (< %.2f m, stuck for %.1f s). "
                "Waiting for map to update or robot to move. "
                "Threshold will relax to %.2f m after 5 seconds.",
                frontiers.size(), min_frontier_distance, stuck_duration.seconds(),
                min_frontier_distance_relaxed);
    return;  // Wait and retry - don't clear blacklist yet
  }
  
  // Use valid frontiers (filtered by distance) instead of all frontiers
  frontiers = valid_frontiers;
  
  // find non blacklisted frontier
  auto frontier =
      std::find_if_not(frontiers.begin(), frontiers.end(),
                       [this](const frontier_exploration::Frontier& f) {
                         return goalOnBlacklist(f.centroid);
                       });
  if (frontier == frontiers.end()) {
    // Before stopping, check if there's still significant unknown space
    // This prevents premature stopping when map is still small or not fully initialized
    double unknown_percentage = 100.0 * unknown_count / map_size;
    double min_unknown_for_exploration = 5.0;  // Require at least 5% unknown space
    
    // Check if we're stuck in a loop with the same frontier
    static int blacklist_clear_count = 0;
    static rclcpp::Time last_blacklist_clear = this->now();
    static geometry_msgs::msg::Point last_cleared_frontier;
    rclcpp::Duration time_since_last_clear = this->now() - last_blacklist_clear;
    
    // Check if we're repeatedly clearing the blacklist for the same frontier
    bool same_frontier_loop = false;
    if (!frontiers.empty() && time_since_last_clear.seconds() < 5.0) {
      double dist_to_last = sqrt(pow(frontiers[0].centroid.x - last_cleared_frontier.x, 2) +
                                  pow(frontiers[0].centroid.y - last_cleared_frontier.y, 2));
      if (dist_to_last < 0.5) {  // Same frontier (within 0.5m)
        same_frontier_loop = true;
        blacklist_clear_count++;
      }
    }
    
    if (same_frontier_loop && blacklist_clear_count >= 3) {
      RCLCPP_ERROR(logger_,
                   "Stuck in loop with same frontier! Clearing blacklist %d times in %.1f seconds. "
                   "Frontier at (%.2f, %.2f) is likely invalid or robot is stuck. "
                   "Blacklisting this frontier permanently and waiting.",
                   blacklist_clear_count, time_since_last_clear.seconds(),
                   frontiers[0].centroid.x, frontiers[0].centroid.y);
      // Add the problematic frontier to blacklist with larger tolerance
      frontier_blacklist_.push_back(frontiers[0].centroid);
      blacklist_clear_count = 0;
      last_blacklist_clear = this->now();
      return;  // Wait and retry - don't clear blacklist
    }
    
    if (unknown_percentage > min_unknown_for_exploration) {
      RCLCPP_WARN(logger_, 
                  "All %zu frontiers are blacklisted, but %.1f%% of map is still unknown. "
                  "Clearing blacklist and retrying. Blacklist size: %zu",
                  frontiers.size(), unknown_percentage, frontier_blacklist_.size());
      // Clear blacklist if there's still significant unknown space
      // This handles cases where frontiers were blacklisted prematurely
      if (!frontiers.empty()) {
        last_cleared_frontier = frontiers[0].centroid;
      }
      frontier_blacklist_.clear();
      last_blacklist_clear = this->now();
      // Retry immediately
      makePlan();
      return;
    } else {
      RCLCPP_INFO(logger_, 
                  "All frontiers traversed/tried out. Unknown space: %.1f%% (threshold: %.1f%%). Stopping exploration.",
                  unknown_percentage, min_unknown_for_exploration);
      RCLCPP_INFO(logger_, "Blacklist size: %zu", frontier_blacklist_.size());
      stop(true);
      return;
    }
  }
  geometry_msgs::msg::Point target_position = frontier->centroid;
  RCLCPP_INFO(logger_, "Selected frontier at (%.2f, %.2f), cost=%.2f, distance=%.2f",
              target_position.x, target_position.y, frontier->cost, frontier->min_distance);

  // Track goal lifecycle to avoid getting stuck when Nav2 doesn't return a result
  static rclcpp::Time last_goal_accepted_time = this->now();
  static geometry_msgs::msg::Point last_goal_target;
  static bool has_active_goal = false;

  // time out if we are not making any progress
  bool same_goal = same_point(prev_goal_, target_position);

  prev_goal_ = target_position;
  if (!same_goal || prev_distance_ > frontier->min_distance) {
    // we have different goal or we made some progress
    last_progress_ = this->now();
    prev_distance_ = frontier->min_distance;
  }
  // black list if we've made no progress for a long time
  // Use durationFromSec to avoid time source mismatch issues
  rclcpp::Duration elapsed = this->now() - last_progress_;
  if (elapsed.seconds() > progress_timeout_ && !resuming_) {
    frontier_blacklist_.push_back(target_position);
    RCLCPP_DEBUG(logger_, "Adding current goal to black list");
    makePlan();
    return;
  }

  // ensure only first call of makePlan was set resuming to true
  if (resuming_) {
    resuming_ = false;
  }

  // IMPORTANT: Don't continuously preempt Nav2 goals from the explorer.
  // However, if Nav2 never reports a result (rare, but happens when the stack is stressed),
  // we must recover, otherwise exploration will appear "stuck" forever.
  if (goal_in_progress_) {
    // When revisiting start, do not run the 30 s watchdog—revisit can take much longer (drive home).
    if (revisit_in_progress_) {
      RCLCPP_INFO_THROTTLE(logger_, *this->get_clock(), 10000,
                           "Revisit to start in progress (watchdog disabled for revisit).");
      return;
    }
    // If we don't have bookkeeping yet (e.g., after restart), initialize it.
    if (!has_active_goal) {
      has_active_goal = true;
      last_goal_accepted_time = this->now();
      last_goal_target = prev_goal_;
    }

    const double max_goal_active_seconds = 30.0;  // watchdog
    rclcpp::Duration active_for = this->now() - last_goal_accepted_time;

    if (active_for.seconds() > max_goal_active_seconds) {
      RCLCPP_WARN(logger_,
                  "Goal appears stuck (active for %.1f s). Canceling Nav2 goal and blacklisting target (%.2f, %.2f).",
                  active_for.seconds(), last_goal_target.x, last_goal_target.y);

      // Cancel all NavigateToPose goals (we don't store the goal handle reliably)
      try {
        move_base_client_->async_cancel_all_goals();
      } catch (...) {
        // Best-effort; even if cancel fails, we'll drop local state to allow recovery
      }

      goal_in_progress_ = false;
      has_active_goal = false;
      frontier_blacklist_.push_back(last_goal_target);
      makePlan();
      return;
    }

    RCLCPP_INFO_THROTTLE(logger_, *this->get_clock(), 5000,
                         "Goal in progress (preemption disabled). Active for %.1f s, target (%.2f, %.2f).",
                         active_for.seconds(), last_goal_target.x, last_goal_target.y);
    return;
  }

  RCLCPP_INFO(logger_, "Sending goal to move base nav2: (%.2f, %.2f)",
              target_position.x, target_position.y);

  // send goal to move_base if we have something new to pursue
  auto goal = nav2_msgs::action::NavigateToPose::Goal();
  goal.pose.pose.position = target_position;
  goal.pose.pose.orientation.w = 1.;
  goal.pose.header.frame_id = costmap_client_.getGlobalFrameID();
  goal.pose.header.stamp = this->now();

  auto send_goal_options =
      rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
  send_goal_options.goal_response_callback =
      [this, target_position](const NavigationGoalHandle::SharedPtr& goal_handle) {
        if (!goal_handle) {
          RCLCPP_WARN(logger_, "Goal was rejected by Nav2!");
          goal_in_progress_ = false;
          has_active_goal = false;
          return;
        }
        goal_in_progress_ = true;
        // Update watchdog bookkeeping on accept
        has_active_goal = true;
        last_goal_accepted_time = this->now();
        last_goal_target = target_position;
        RCLCPP_INFO(logger_, "Goal accepted by Nav2, navigating to (%.2f, %.2f)",
                    target_position.x, target_position.y);
      };
  // send_goal_options.feedback_callback =
  //   std::bind(&Explore::feedback_callback, this, _1, _2);
  send_goal_options.result_callback =
      [this,
       target_position](const NavigationGoalHandle::WrappedResult& result) {
        reachedGoal(result, target_position);
      };
  move_base_client_->async_send_goal(goal, send_goal_options);

  // Set watchdog start point optimistically; updated on accept callback as well
  has_active_goal = true;
  last_goal_accepted_time = this->now();
  last_goal_target = target_position;
}

void Explore::returnToInitialPose()
{
  RCLCPP_INFO(logger_, "Returning to initial pose.");
  auto goal = nav2_msgs::action::NavigateToPose::Goal();
  goal.pose.pose.position = initial_pose_.position;
  goal.pose.pose.orientation = initial_pose_.orientation;
  goal.pose.header.frame_id = costmap_client_.getGlobalFrameID();
  goal.pose.header.stamp = this->now();

  auto send_goal_options =
      rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
  move_base_client_->async_send_goal(goal, send_goal_options);
}

void Explore::sendRevisitGoal()
{
  RCLCPP_INFO(logger_, "Revisit: navigating to start at (%.2f, %.2f) for loop-closure opportunity.",
              initial_pose_.position.x, initial_pose_.position.y);
  revisit_in_progress_ = true;
  auto goal = nav2_msgs::action::NavigateToPose::Goal();
  goal.pose.pose.position = initial_pose_.position;
  goal.pose.pose.orientation = initial_pose_.orientation;
  goal.pose.header.frame_id = costmap_client_.getGlobalFrameID();
  goal.pose.header.stamp = this->now();

  auto send_goal_options =
      rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
  send_goal_options.goal_response_callback =
      [this](const NavigationGoalHandle::SharedPtr& goal_handle) {
        if (!goal_handle) {
          RCLCPP_WARN(logger_, "Revisit goal was rejected by Nav2!");
          goal_in_progress_ = false;
          revisit_in_progress_ = false;
          makePlan();
          return;
        }
        goal_in_progress_ = true;
      };
  send_goal_options.result_callback =
      [this](const NavigationGoalHandle::WrappedResult& result) {
        reachedGoal(result, initial_pose_.position);
      };
  move_base_client_->async_send_goal(goal, send_goal_options);
  goal_in_progress_ = true;
}

bool Explore::goalOnBlacklist(const geometry_msgs::msg::Point& goal)
{
  // Increased tolerance to 0.3m (was 5 cells * 0.05m = 0.25m)
  // This helps prevent selecting frontiers that are essentially the same location
  constexpr static double tolerance_meters = 0.3;

  // check if a goal is on the blacklist for goals that we're pursuing
  for (auto& frontier_goal : frontier_blacklist_) {
    double x_diff = fabs(goal.x - frontier_goal.x);
    double y_diff = fabs(goal.y - frontier_goal.y);
    double distance = sqrt(x_diff * x_diff + y_diff * y_diff);

    // Use distance-based check instead of separate x/y checks for better accuracy
    if (distance < tolerance_meters) {
      return true;
    }
  }
  return false;
}

void Explore::reachedGoal(const NavigationGoalHandle::WrappedResult& result,
                          const geometry_msgs::msg::Point& frontier_goal)
{
  goal_in_progress_ = false;
  if (revisit_in_progress_) {
    revisit_in_progress_ = false;
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      goals_since_revisit_ = 0;
      RCLCPP_INFO(logger_, "Revisit to start completed. Resuming exploration.");
    } else {
      RCLCPP_WARN(logger_, "Revisit goal ended with code %d. Resuming exploration.", static_cast<int>(result.code));
    }
    makePlan();
    return;
  }
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(logger_, "Goal reached successfully at (%.2f, %.2f). Planning next frontier...",
                  frontier_goal.x, frontier_goal.y);
      // If the map isn't updating quickly (or the frontier doesn't disappear),
      // blacklist this reached frontier so we move on.
      frontier_blacklist_.push_back(frontier_goal);
      goals_since_revisit_++;
      RCLCPP_INFO(logger_, "Blacklisting reached frontier (%.2f, %.2f). Blacklist size: %zu",
                  frontier_goal.x, frontier_goal.y, frontier_blacklist_.size());
      if (revisit_enabled_) {
        RCLCPP_INFO(logger_, "Goals since last revisit: %d (revisit every %d).",
                    goals_since_revisit_, revisit_after_n_goals_);
      }
      if (revisit_enabled_ && goals_since_revisit_ >= revisit_after_n_goals_) {
        RCLCPP_INFO(logger_, "Going for periodic revisit: %d goals since last revisit (threshold %d). Navigating to start at (%.2f, %.2f).",
                    goals_since_revisit_, revisit_after_n_goals_,
                    initial_pose_.position.x, initial_pose_.position.y);
        sendRevisitGoal();
        return;
      }
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_DEBUG(logger_, "Goal was aborted");
      frontier_blacklist_.push_back(frontier_goal);
      RCLCPP_DEBUG(logger_, "Adding current goal to black list");
      // If it was aborted probably because we've found another frontier goal,
      // so just return and don't make plan again
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_DEBUG(logger_, "Goal was canceled");
      // If goal canceled might be because exploration stopped from topic. Don't make new plan.
      return;
    default:
      RCLCPP_WARN(logger_, "Unknown result code from move base nav2");
      break;
  }
  // find new goal immediately regardless of planning frequency.
  // execute via timer to prevent dead lock in move_base_client (this is
  // callback for sendGoal, which is called in makePlan). the timer must live
  // until callback is executed.
  // oneshot_ = relative_nh_.createTimer(
  //     ros::Duration(0, 0), [this](const ros::TimerEvent&) { makePlan(); },
  //     true);

  // Because of the 1-thread-executor nature of ros2 I think timer is not
  // needed.
  makePlan();
}

void Explore::start()
{
  RCLCPP_INFO(logger_, "Exploration started.");
}

void Explore::stop(bool finished_exploring)
{
  RCLCPP_INFO(logger_, "Exploration stopped.");
  move_base_client_->async_cancel_all_goals();
  exploring_timer_->cancel();

  if (return_to_init_ && finished_exploring) {
    returnToInitialPose();
  }
}

void Explore::resume()
{
  resuming_ = true;
  RCLCPP_INFO(logger_, "Exploration resuming.");
  // Reactivate the timer
  exploring_timer_->reset();
  // Resume immediately
  makePlan();
}

}  // namespace explore

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  // ROS1 code
  /*
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  } */
  rclcpp::spin(
      std::make_shared<explore::Explore>());  // std::move(std::make_unique)?
  rclcpp::shutdown();
  return 0;
}
