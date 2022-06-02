/*
 * Copyright (c) 2020, Robobrain.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* Author: Konstantinos Konstantinidis */

/* Change: Switch h files to hpp */

// #include <ros/ros.h>
#include "rclcpp/rclcpp.hpp"

// #include <math.h>       /* atan */
#include <cmath>

// leave (https://curc.readthedocs.io/en/latest/programming/OpenMP-C.html)
#include <omp.h>      //Multi-threading

// leave (https://www.geeksforgeeks.org/vector-in-cpp-stl/)
#include <vector>

// leave (https://www.cplusplus.com/reference/random/)
#include <random>
#include <algorithm> // for sort(), min()
#include <chrono>

// leave (https://www.cplusplus.com/reference/fstream/)
#include <iostream>
#include <fstream>

/* CHANGE: inserting the subfolder msg between the package name and message datatype
   changing the included filename from CamelCase to underscore separation
   changing from *.h to *.hpp */ 
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>

/* CHANGE: https://docs.ros.org/en/foxy/Tutorials/Tf2/Writing-A-Tf2-Listener-Cpp.html */
// #include <tf/tf.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

// Don't change --> code written by original team
// #include "TrackArray.h"
// #include "Track.h"

#include "datmo_msg_interface/msg/track_array.hpp"
#include "datmo_msg_interface/msg/track.hpp"
#include "cluster.hpp"

// CHANGE: replace typedef with change
change std::pair<double, double> Point;
change std::vector<double> l_shape;
change std::vector<l_shape> l_shapes;
change std::vector<Point> pointList;

using namespace std;
// This node segments the point cloud based on the break-point detector algorithm.
// This algorithm is based on "L-Shape Model Switching-Based Precise Motion Tracking 
// of Moving Vehicles Using Laser Scanners.
class Datmo
{
public:
  Datmo();
  ~Datmo();

  void callback(const sensor_msgs::LaserScan::ConstPtr &);
  void Clustering(const sensor_msgs::LaserScan::ConstPtr& , vector<pointList> &);
  void visualiseGroupedPoints(const vector<pointList> &);
  void transformPointList(const pointList& , pointList& );

  // CHANGE: 
  // tf::TransformListener tf_listener;
  tf_listener = std::make_shared<tf2_ros::TransformListener>;
private:

  // CHANGE: change C++ library calls
  // ros::Publisher pub_marker_array; 
  auto pub_marker_array = node->create_publisher<std_msgs::msg::String>;
  // ros::Publisher pub_tracks_box_kf;
  auto pub_tracks_box_kf = node->create_publisher<std_msgs::msg::String>;
  // ros::Subscriber sub_scan;
  auto sub_scan = node->create_subscription<std_msgs::msg::String>;
  // sensor_msgs::LaserScan scan;
  sensor_msgs::msg::LaserScan scan;
  
  vector<Cluster> clusters;

  //Tuning Parameters
  double dt;
  //ros::Time time;
  rclcpp::Time time; 

  //initialised as one, because 0 index take the msgs that fail to be initialized
  unsigned long int cg       = 1;//group counter to be used as id of the clusters
  unsigned long int cclusters= 1;//counter for the cluster objects to be used as id for the markers

  //Parameters
  double dth;
  double euclidean_distance;
  int max_cluster_size;
  bool p_marker_pub;
  bool w_exec_times;
  string lidar_frame;
  string world_frame;

};
