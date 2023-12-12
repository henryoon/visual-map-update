// /*******************************************************************************
//  *      Title     : pose_tracking_example.cpp
//  *      Project   : moveit_servo
//  *      Created   : 09/04/2020
//  *      Author    : Adam Pettinger
//  *
//  * BSD 3-Clause License
//  *
//  * Copyright (c) 2019, Los Alamos National Security, LLC
//  * All rights reserved.
//  *
//  * Redistribution and use in source and binary forms, with or without
//  * modification, are permitted provided that the following conditions are met:
//  *
//  * * Redistributions of source code must retain the above copyright notice, this
//  *   list of conditions and the following disclaimer.
//  *
//  * * Redistributions in binary form must reproduce the above copyright notice,
//  *   this list of conditions and the following disclaimer in the documentation
//  *   and/or other materials provided with the distribution.
//  *
//  * * Neither the name of the copyright holder nor the names of its
//  *   contributors may be used to endorse or promote products derived from
//  *   this software without specific prior written permission.
//  *
//  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
//  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//  * ARE
//  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
//  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
//  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
//  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
//  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//  *******************************************************************************/

// #include <std_msgs/Int8.h>
// #include <geometry_msgs/TransformStamped.h>

// #include <moveit_servo/servo.h>
// #include <moveit_servo/pose_tracking.h>
// #include <moveit_servo/status_codes.h>
// #include <moveit_servo/make_shared_from_pool.h>
// #include <thread>

// static const std::string LOGNAME = "cpp_interface_example";

// // Class for monitoring status of moveit_servo
// class StatusMonitor
// {
// public:
//   StatusMonitor(ros::NodeHandle& nh, const std::string& topic)
//   {
//     sub_ = nh.subscribe(topic, 1, &StatusMonitor::statusCB, this);
//   }

// private:
//   void statusCB(const std_msgs::Int8ConstPtr& msg)
//   {
//     moveit_servo::StatusCode latest_status = static_cast<moveit_servo::StatusCode>(msg->data);
//     if (latest_status != status_)
//     {
//       status_ = latest_status;
//       const auto& status_str = moveit_servo::SERVO_STATUS_CODE_MAP.at(status_);
//       ROS_INFO_STREAM_NAMED(LOGNAME, "Servo status: " << status_str);
//     }
//   }
//   moveit_servo::StatusCode status_ = moveit_servo::StatusCode::INVALID;
//   ros::Subscriber sub_;
// };

// /**
//  * Instantiate the pose tracking interface.
//  * Send a pose slightly different from the starting pose
//  * Then keep updating the target pose a little bit
//  */
// int main(int argc, char** argv)
// {
//   ros::init(argc, argv, LOGNAME);
//   ros::NodeHandle nh("~");
//   ros::AsyncSpinner spinner(8);
//   spinner.start();

//   // Load the planning scene monitor
//   planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor;
//   planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
//   if (!planning_scene_monitor->getPlanningScene())
//   {
//     ROS_ERROR_STREAM_NAMED(LOGNAME, "Error in setting up the PlanningSceneMonitor.");
//     exit(EXIT_FAILURE);
//   }

//   planning_scene_monitor->startSceneMonitor();
//   planning_scene_monitor->startWorldGeometryMonitor(
//       planning_scene_monitor::PlanningSceneMonitor::DEFAULT_COLLISION_OBJECT_TOPIC,
//       planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_WORLD_TOPIC,
//       false /* skip octomap monitor */);
//   planning_scene_monitor->startStateMonitor();

//   // Create the pose tracker
//   moveit_servo::PoseTracking tracker(nh, planning_scene_monitor);

//   // Make a publisher for sending pose commands
//   ros::Publisher target_pose_pub =
//       nh.advertise<geometry_msgs::PoseStamped>("target_pose", 1 /* queue */, true /* latch */);

//   // Subscribe to servo status (and log it when it changes)
//   StatusMonitor status_monitor(nh, "status");

//   Eigen::Vector3d lin_tol{ 0.0001, 0.0001, 0.0001 };
//   double rot_tol = 0.1;

//   // Get the current EE transform
//   geometry_msgs::TransformStamped current_ee_tf;
//   tracker.getCommandFrameTransform(current_ee_tf);

//   // Convert it to a Pose
//   geometry_msgs::PoseStamped target_pose;
//   target_pose.header.frame_id = current_ee_tf.header.frame_id;
//   target_pose.pose.position.x = current_ee_tf.transform.translation.x;
//   target_pose.pose.position.y = current_ee_tf.transform.translation.y;
//   target_pose.pose.position.z = current_ee_tf.transform.translation.z;
//   target_pose.pose.orientation = current_ee_tf.transform.rotation;

//   // Modify it a little bit
//   // target_pose.pose.position.x += 0.1;
//   // target_pose.pose.position.y += 0.1;

//   // resetTargetPose() can be used to clear the target pose and wait for a new one, e.g. when moving between multiple
//   // waypoints
//   // tracker.resetTargetPose();

//   // Publish target pose
//   // target_pose.header.stamp = ros::Time::now();
//   // target_pose_pub.publish(target_pose);

//   // Run the pose tracking in a new thread
//   std::thread move_to_pose_thread(
//       [&tracker, &lin_tol, &rot_tol] { tracker.moveToPose(lin_tol, rot_tol, 0.1 /* target pose timeout */); });

//   ros::Rate loop_rate(500);
//   for (size_t i = 0; i < 2; ++i)
//   {
  
//     if (i <200)
//     target_pose.pose.position.z -= 0.01;
//     // std::cout << target_pose.pose.position << std::endl;
//     target_pose.header.stamp = ros::Time::now();
//     target_pose_pub.publish(target_pose);

//     loop_rate.sleep();
//   }
//   while (true)
//   {
//     printf("move\n");
//     loop_rate.sleep();
//   }

//   // Make sure the tracker is stopped and clean up
//   tracker.stopMotion();
//   move_to_pose_thread.join();

//   return EXIT_SUCCESS;
// }













// /*******************************************************************************
//  *      Title     : pose_tracking_example.cpp
//  *      Project   : moveit_servo
//  *      Created   : 09/04/2020
//  *      Author    : Adam Pettinger
//  *
//  * BSD 3-Clause License
//  *
//  * Copyright (c) 2019, Los Alamos National Security, LLC
//  * All rights reserved.
//  *
//  * Redistribution and use in source and binary forms, with or without
//  * modification, are permitted provided that the following conditions are met:
//  *
//  * * Redistributions of source code must retain the above copyright notice, this
//  *   list of conditions and the following disclaimer.
//  *
//  * * Redistributions in binary form must reproduce the above copyright notice,
//  *   this list of conditions and the following disclaimer in the documentation
//  *   and/or other materials provided with the distribution.
//  *
//  * * Neither the name of the copyright holder nor the names of its
//  *   contributors may be used to endorse or promote products derived from
//  *   this software without specific prior written permission.
//  *
//  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
//  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//  * ARE
//  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
//  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
//  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
//  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
//  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//  *******************************************************************************/

#include <std_msgs/Int8.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Float64MultiArray.h>

#include <moveit_servo/servo.h>
#include <moveit_servo/pose_tracking.h>
#include <moveit_servo/status_codes.h>
#include <moveit_servo/make_shared_from_pool.h>
#include <thread>
#include <tf2/LinearMath/Quaternion.h>

#define PI 3.141592

static const std::string LOGNAME = "cpp_interface_example";

// Class for monitoring status of moveit_servo
class StatusMonitor
{
public:
  StatusMonitor(ros::NodeHandle& nh, const std::string& topic)
  {
    sub_ = nh.subscribe(topic, 1, &StatusMonitor::statusCB, this);
  }

private:
  void statusCB(const std_msgs::Int8ConstPtr& msg)
  {
    moveit_servo::StatusCode latest_status = static_cast<moveit_servo::StatusCode>(msg->data);
    if (latest_status != status_)
    {
      status_ = latest_status;
      const auto& status_str = moveit_servo::SERVO_STATUS_CODE_MAP.at(status_);
      ROS_INFO_STREAM_NAMED(LOGNAME, "Servo status: " << status_str);
    }
  }
  moveit_servo::StatusCode status_ = moveit_servo::StatusCode::INVALID;
  ros::Subscriber sub_;

};

struct euler {
  float roll;
  float pitch;
  float yaw;
};

struct euler quaterniontoeuler(float qx, float qy, float qz, float qw)
{
  struct euler E;
  float pitch;

  E.roll = atan2(2* (qw * qw + qy *qz), 1 - 2 * (qx * qx + qy * qy));
  pitch = 2 * (qw * qy - qz * qx);
  if (abs(pitch) >= 1)
  {
    E.pitch = copysign(PI / 2, pitch);
  }
  else
  {
    E.pitch = asin(pitch);
  }
  E.yaw = atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz));
  return E;
}

struct quaternion {
  float qx;
  float qy;
  float qz;
  float qw;
};

struct quaternion eulertoquaternion(float roll, float pitch, float yaw)
{
  struct quaternion Q;
  Q.qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2);
  Q.qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2);
  Q.qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2);
  Q.qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2);
  return Q;
}

/**;
 * Instantiate the pose tracking interface.
 * Send a pose slightly different from the starting pose
 * Then keep updating the target pose a little bit
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, LOGNAME);
  double start, end;
  struct euler eef_euler;
  struct quaternion eef_quaternion;
  ros::NodeHandle nh("~");
  ros::AsyncSpinner spinner(8);
  spinner.start();

  // Load the planning scene monitor
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor;
  planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
  if (!planning_scene_monitor->getPlanningScene())
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Error in setting up the PlanningSceneMonitor.");
    exit(EXIT_FAILURE);
  }

  planning_scene_monitor->startSceneMonitor();
  planning_scene_monitor->startWorldGeometryMonitor(
      planning_scene_monitor::PlanningSceneMonitor::DEFAULT_COLLISION_OBJECT_TOPIC,
      planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_WORLD_TOPIC,
      false /* skip octomap monitor */);
  planning_scene_monitor->startStateMonitor();

  // Create the pose tracker
  moveit_servo::PoseTracking tracker(nh, planning_scene_monitor);

  // Make a publisher for sending pose commands
  ros::Publisher target_pose_pub =
      nh.advertise<geometry_msgs::PoseStamped>("target_pose", 1 /* queue */, true /* latch */);

  ros::Publisher joint_vel_pub =
      nh.advertise<std_msgs::Float64MultiArray>("joint_vel", 1 /* queue */, true /* latch */);

  // Subscribe to servo status (and log it when it changes)
  StatusMonitor status_monitor(nh, "status");

  Eigen::Vector3d lin_tol{ 0.000001, 0.000001, 0.000001 };
  double rot_tol = 0.000001;

  // Get the current EE transform
  geometry_msgs::TransformStamped current_ee_tf;
  // for robot_link_command_frame == tool0
  // tracker.getCommandFrameTransform(current_ee_tf);

  // for robot_link_command_frame == base_link
  tracker.getEEFrameTransform(current_ee_tf);

  std::cout << current_ee_tf << std::endl;
  // Convert it to a Pose
  geometry_msgs::PoseStamped target_pose;
  target_pose.header.frame_id = current_ee_tf.header.frame_id;
  target_pose.pose.position.x = current_ee_tf.transform.translation.x;
  target_pose.pose.position.y = current_ee_tf.transform.translation.y;
  target_pose.pose.position.z = current_ee_tf.transform.translation.z;
  target_pose.pose.orientation = current_ee_tf.transform.rotation;

  std_msgs::Float64MultiArray joint_vel;


  // // Modify it a little bit
  // target_pose.pose.position.x += 0.01;

  // // resetTargetPose() can be used to clear the target pose and wait for a new one, e.g. when moving between multiple
  // // waypoints
  // tracker.resetTargetPose();

  // // Publish target pose
  // target_pose.header.stamp = ros::Time::now();
  // target_pose_pub.publish(target_pose);

  // Run the pose tracking in a new thread
  std::thread move_to_pose_thread(
      [&tracker, &lin_tol, &rot_tol] { tracker.moveToPose(lin_tol, rot_tol, 0.1 /* target pose timeout */); });

  ros::Rate loop_rate(100);
  start = (double)clock() / CLOCKS_PER_SEC;    
  // for (size_t i = 0; i < 500; ++i)
  // {
  //   // Modify the pose target a little bit each cycle
  //   // This is a dynamic pose target
  //   eef_euler = quaterniontoeuler(target_pose.pose.orientation.x, target_pose.pose.orientation.y, target_pose.pose.orientation.z, target_pose.pose.orientation.w);
  //   eef_euler.yaw = eef_euler.yaw + 0.01;
  //   eef_quaternion = eulertoquaternion(eef_euler.roll, eef_euler.pitch, eef_euler.yaw);
  //   // printf("%f\t%f\t%f\t%f\n", eef_quaternion.qx, eef_quaternion.qy, eef_quaternion.qz, eef_quaternion.qw);
  //   // printf("%f\t%f\n",target_pose.pose.position.x,target_pose.pose.position.y);
  //   target_pose.pose.position.x = current_ee_tf.transform.translation.x;
  //   target_pose.pose.position.y = current_ee_tf.transform.translation.y;
  //   target_pose.pose.position.z = current_ee_tf.transform.translation.z;
  //   target_pose.pose.orientation.x = eef_quaternion.qx;
  //   target_pose.pose.orientation.y = eef_quaternion.qy;
  //   target_pose.pose.orientation.z = eef_quaternion.qz;
  //   target_pose.pose.orientation.w = eef_quaternion.qw;

  //   target_pose.header.stamp = ros::Time::now();
  //   target_pose_pub.publish(target_pose);

  //   loop_rate.sleep();
  // }
  for (size_t i = 0; i < 200; ++i)
  {
    // Modify the pose target a little bit each cycle
    // This is a dynamic pose target
    target_pose.pose.position.z += 0.0001;
    // target_pose.pose.position.z = 0.6;
    target_pose.header.stamp = ros::Time::now();
    target_pose_pub.publish(target_pose);

    loop_rate.sleep();
  }
  while(true)
  {
    // printf("move\n");
    // std::cout << current_ee_tf << std::endl;
    // printf("position: %f, %f, %f\n", target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z);
    // printf("orientation: %f, %f, %f, %f\n", target_pose.pose.orientation.x, target_pose.pose.orientation.y, target_pose.pose.orientation.z, target_pose.pose.orientation.w);
    loop_rate.sleep();
  }
  end = (((double)clock()) / CLOCKS_PER_SEC);
  printf("time :%lf\n", (end-start));
  // while (true)
  // {
  //   loop_rate.sleep();
  // }

  // Make sure the tracker is stopped and clean up
  tracker.stopMotion();
  // move_to_pose_thread.join();
  // joint_vel.data.push_back(0);
  // joint_vel.data.push_back(0);
  // joint_vel.data.push_back(0);
  // joint_vel.data.push_back(0);
  // joint_vel.data.push_back(0);
  // joint_vel.data.push_back(0); 
  // joint_vel_pub.publish(joint_vel);
  printf("end\n");

  return EXIT_SUCCESS;
}

/*******************************************************************************
 *      Title     : pose_tracking_example.cpp
 *      Project   : moveit_servo
 *      Created   : 09/04/2020
 *      Author    : Adam Pettinger
 *
 * BSD 3-Clause License
 *
 * Copyright (c) 2019, Los Alamos National Security, LLC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

// #include <std_msgs/Int8.h>
// #include <geometry_msgs/TransformStamped.h>

// #include <moveit_servo/servo.h>
// #include <moveit_servo/pose_tracking.h>
// #include <moveit_servo/status_codes.h>
// #include <moveit_servo/make_shared_from_pool.h>
// #include <thread>

// static const std::string LOGNAME = "cpp_interface_example";

// // Class for monitoring status of moveit_servo
// class StatusMonitor
// {
// public:
//   StatusMonitor(ros::NodeHandle& nh, const std::string& topic)
//   {
//     sub_ = nh.subscribe(topic, 1, &StatusMonitor::statusCB, this);
//   }

// private:
//   void statusCB(const std_msgs::Int8ConstPtr& msg)
//   {
//     moveit_servo::StatusCode latest_status = static_cast<moveit_servo::StatusCode>(msg->data);
//     if (latest_status != status_)
//     {
//       status_ = latest_status;
//       const auto& status_str = moveit_servo::SERVO_STATUS_CODE_MAP.at(status_);
//       ROS_INFO_STREAM_NAMED(LOGNAME, "Servo status: " << status_str);
//     }
//   }
//   moveit_servo::StatusCode status_ = moveit_servo::StatusCode::INVALID;
//   ros::Subscriber sub_;
// };

// /**
//  * Instantiate the pose tracking interface.
//  * Send a pose slightly different from the starting pose
//  * Then keep updating the target pose a little bit
//  */
// int main(int argc, char** argv)
// {
//   ros::init(argc, argv, LOGNAME);
//   ros::NodeHandle nh("~");
//   ros::AsyncSpinner spinner(8);
//   spinner.start();

//   // Load the planning scene monitor
//   planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor;
//   planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
//   if (!planning_scene_monitor->getPlanningScene())
//   {
//     ROS_ERROR_STREAM_NAMED(LOGNAME, "Error in setting up the PlanningSceneMonitor.");
//     exit(EXIT_FAILURE);
//   }

//   planning_scene_monitor->startSceneMonitor();
//   planning_scene_monitor->startWorldGeometryMonitor(
//       planning_scene_monitor::PlanningSceneMonitor::DEFAULT_COLLISION_OBJECT_TOPIC,
//       planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_WORLD_TOPIC,
//       false /* skip octomap monitor */);
//   planning_scene_monitor->startStateMonitor();

//   // Create the pose tracker
//   moveit_servo::PoseTracking tracker(nh, planning_scene_monitor);

//   // Make a publisher for sending pose commands
//   ros::Publisher target_pose_pub =
//       nh.advertise<geometry_msgs::PoseStamped>("target_pose", 1 /* queue */, true /* latch */);

//   // Subscribe to servo status (and log it when it changes)
//   StatusMonitor status_monitor(nh, "status");

//   Eigen::Vector3d lin_tol{ 0.000001, 0.000001, 0.000001 };
//   double rot_tol = 0.1;

//   // Get the current EE transform
//   geometry_msgs::TransformStamped current_ee_tf;
//   tracker.getCommandFrameTransform(current_ee_tf);

//   // Convert it to a Pose
//   geometry_msgs::PoseStamped target_pose;
//   target_pose.header.frame_id = current_ee_tf.header.frame_id;
//   target_pose.pose.position.x = current_ee_tf.transform.translation.x;
//   target_pose.pose.position.y = current_ee_tf.transform.translation.y;
//   target_pose.pose.position.z = current_ee_tf.transform.translation.z;
//   target_pose.pose.orientation = current_ee_tf.transform.rotation;

//   // // Modify it a little bit
//   // target_pose.pose.position.x += 0.1;

//   // // resetTargetPose() can be used to clear the target pose and wait for a new one, e.g. when moving between multiple
//   // // waypoints
//   // tracker.resetTargetPose();

//   // // Publish target pose
//   // target_pose.header.stamp = ros::Time::now();
//   // target_pose_pub.publish(target_pose);

//   // Run the pose tracking in a new thread
//   std::thread move_to_pose_thread(
//       [&tracker, &lin_tol, &rot_tol] { tracker.moveToPose(lin_tol, rot_tol, 0.1 /* target pose timeout */); });

//   ros::Rate loop_rate(100);
//   for (size_t i = 0; i < 500; ++i)
//   {
//     // Modify the pose target a little bit each cycle
//     // This is a dynamic pose target
//     if(i<150)
//     {
//       target_pose.pose.position.z += 0.0004;
//     }
//     else
//     {
//       target_pose.pose.position.z -= 0.0004;
//     }
//     target_pose.header.stamp = ros::Time::now();
//     target_pose_pub.publish(target_pose);

//     loop_rate.sleep();
//   }
//   // Make sure the tracker is stopped and clean up
//   tracker.stopMotion();
//   move_to_pose_thread.join();

//   return EXIT_SUCCESS;
// }