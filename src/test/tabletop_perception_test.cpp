/*
 *  Test the tabletop perception system.
 *
 *  Author: David Butterworth
 *  Originally based on code from a ROS Tutorial.
 */

/*
 * Copyright (c) 2013, David Butterworth, PAL Robotics S.L.
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

#include <ros/ros.h>

#include <tabletop_object_detector/TabletopDetection.h>
#include <tabletop_collision_map_processing/TabletopCollisionMapProcessing.h>

#include <arm_navigation_msgs/GetPlanningScene.h>

#include <tf/transform_listener.h>    // getTableHeight(), getNearestObjectIndex()
#include <tf/transform_broadcaster.h> // getTableHeight()

// REEM head scan snapshotter Action Client
#include <actionlib/client/simple_action_client.h>
#include <pr2_controllers_msgs/PointHeadAction.h>

// Return index number of graspable object that is nearest to reference point
// and publish TF marker in Rviz.
// Based on PR2 x-y nearest & Martin Gunther's xyz nearest
tf::TransformListener *tf_listener;
tf::TransformBroadcaster *tf_broadcaster; // just a pointer, so we can use it before ROS::Init
bool getNearestObjectIndex(std::vector<object_manipulation_msgs::GraspableObject>& objects,
                           geometry_msgs::PointStamped& reference_point, int& object_ind);

bool getTableHeight(geometry_msgs::PoseStamped& _table_pose, double& table_height);

typedef actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction> PointHeadClient; // for head scan snapshotter Action
PointHeadClient* point_head_client_;

int main(int argc, char **argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "tabletop_perception_test");
    ros::NodeHandle nh;

    // Read the parameter for min_marker_quality
    // Object detection results above this quality are published as /tabletop_object_recognition_markers
    double min_marker_quality_;
    nh.param<double>("/tabletop_object_recognition/min_marker_quality", min_marker_quality_, 0.003);

    // Set service and action names
    const std::string OBJECT_DETECTION_SERVICE_NAME = "/object_detection";
    const std::string COLLISION_PROCESSING_SERVICE_NAME = "/tabletop_collision_map_processing/tabletop_collision_map_processing";
    static const std::string SET_PLANNING_SCENE_DIFF_NAME = "/environment_server/set_planning_scene_diff";

    // Create service and action clients
    ros::ServiceClient object_detection_srv;
    ros::ServiceClient collision_processing_srv;
    ros::ServiceClient get_planning_scene_client;

    // Create TF listener & broadcaster, for nearest_object()
    tf_listener = new tf::TransformListener();
    tf_broadcaster = new tf::TransformBroadcaster();

    // Wait for Object Detection client
    while (!ros::service::waitForService(OBJECT_DETECTION_SERVICE_NAME, ros::Duration(2.0)) && nh.ok() )
    {
        ROS_INFO("Waiting for object detection service to come up");
    }
    if (!nh.ok()) exit(0);
    object_detection_srv = nh.serviceClient<tabletop_object_detector::TabletopDetection>(OBJECT_DETECTION_SERVICE_NAME, true);

    // Wait for Collision Map Processing client
    while ( !ros::service::waitForService(COLLISION_PROCESSING_SERVICE_NAME, ros::Duration(2.0)) && nh.ok() )
    {
        ROS_INFO("Waiting for collision processing service to come up");
    }
    if (!nh.ok()) exit(0);
    collision_processing_srv = nh.serviceClient<tabletop_collision_map_processing::TabletopCollisionMapProcessing>
                               (COLLISION_PROCESSING_SERVICE_NAME, true); // true = persistant service

    // Wait for Planning Scene client
    while (!ros::service::waitForService(SET_PLANNING_SCENE_DIFF_NAME, ros::Duration(2.0)) && nh.ok() )
    {
        ROS_INFO("Waiting for Planning Scene service to come up");
    }
    get_planning_scene_client = nh.serviceClient<arm_navigation_msgs::GetPlanningScene>(SET_PLANNING_SCENE_DIFF_NAME);

    // Initialize the client for the Action interface & spin a thread
    point_head_client_ = new PointHeadClient("/head_traj_controller/head_scan_snapshot_action", true);

    // Wait for the action server to come up
    while(!point_head_client_->waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the '/head_traj_controller/head_scan_snapshot_action' Action Server to come up");
    }


    // Scan the table using PointCloud snapshotter
    pr2_controllers_msgs::PointHeadGoal scan_table;
    ROS_INFO("Sending scan table goal...");
    point_head_client_->sendGoal(scan_table);
    point_head_client_->waitForResult();
    if(point_head_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("done!");
    }
    else
    {
        ROS_INFO("The release action failed.");
    }


    // Tabletop Object Detection

    ROS_INFO("Calling tabletop detector");
    tabletop_object_detector::TabletopDetection detection_call;

    // Return the pointcloud (cluster) for each detected object
    // if this is false, you have no data to pass to tabletop_collision_map_processing node
    detection_call.request.return_clusters = true;

    // Return matching object models
    // Each cluster will have 1 or more models, even if the confidence rating is too low to display a marker in Rviz
    // If the household objects database is not connected, the model list will be empty
    detection_call.request.return_models = true;
    // Number of models to return for each object cluster
    // If there is more than 1 model with high confidence, they will all be displayed in Rviz
    detection_call.request.num_models = 1;

    if (!object_detection_srv.call(detection_call))
    {
        ROS_ERROR("Tabletop detection service failed");
        return -1;
    }
    if (detection_call.response.detection.result != detection_call.response.detection.SUCCESS)
    {
        std::string error_description;
        switch (detection_call.response.detection.result)
        {
        case 1:
            error_description = "NO_CLOUD_RECEIVED";
            break;
        case 2:
            error_description = "NO_TABLE";
            break;
        case 3:
            error_description = "OTHER_ERROR";
            break;
        case 4:
            error_description = "SUCCESS";
            break;
        default:
            error_description = "unknown error code";
            break;
        }
        ROS_ERROR("Tabletop detection returned error code %d (%s)", detection_call.response.detection.result, error_description.c_str() );
        return -1;
    }

    ROS_INFO("Tabletop detection succeeded:   x_min: %f  x_max: %f   y_min: %f  y_max: %f",
             detection_call.response.detection.table.x_min, detection_call.response.detection.table.x_max,
             detection_call.response.detection.table.y_min, detection_call.response.detection.table.y_max );

    if (detection_call.response.detection.clusters.empty() && detection_call.response.detection.models.empty() )
    {
        if (detection_call.request.return_clusters == true)
        {
            ROS_ERROR("The tabletop detector detected the table, but found no objects. ");
            return -1;
        }
        else
        {
            ROS_ERROR("Object detection suceeded, but cluster data was not requested. ");
            return -1;
        }
    }
    else
    {
        ROS_INFO("Object detection suceeded. Found %d unknown objects (clusters).", detection_call.response.detection.clusters.size() );

        // If return_models is true, the models list will contain one entry for each cluster, even if no objects were matched!
        if (!detection_call.response.detection.models.empty())
        {
            ROS_INFO("Model detection was requested, ");
        }
        else
        {
            ROS_INFO("Model detection not requested. ");
        }

        // For each detected object (cluster), print the matching models
        for (unsigned int i=0; i < detection_call.response.detection.clusters.size(); i++)
        {
            ROS_INFO("   Cluster %d has a cloud with %d points ", i, detection_call.response.detection.clusters[i].points.size() );

            if (!detection_call.response.detection.models.empty())
            {
                for (unsigned int j=0; j < detection_call.response.detection.models[i].model_list.size(); j++)
                {
                    std::string marker_status = "";
                    if (detection_call.response.detection.models[i].model_list[j].confidence < min_marker_quality_)
                    {
                        marker_status = " (Mesh marker published in Rviz)";
                    }
                    ROS_INFO("     model[%d].model_list[%d] ID %d  Confidence %.4f  %s ", i, j,
                             detection_call.response.detection.models[i].model_list[j].model_id,
                             detection_call.response.detection.models[i].model_list[j].confidence,
                             marker_status.c_str() );
                }
            }
        }
    } // end detection_call...


    // Tabletop Collision Map Processing

    ROS_INFO("Calling collision map processing");
    tabletop_collision_map_processing::TabletopCollisionMapProcessing processing_call;
    // Pass the results from tabletop object detector (table, clusters, models, etc.)
    processing_call.request.detection_result = detection_call.response.detection;
    // Reset exising map and collision models
    //processing_call.request.reset_static_map = true; // not required, static map is from PR2 laser
    processing_call.request.reset_collision_models = true;
    processing_call.request.reset_attached_models = true;
    //ask for a new static collision map to be taken with the laser
    //after the new models are added to the environment
    //processing_call.request.take_static_collision_map = true; // not required, static map is only for PR2 laser
    // Reference frame
    processing_call.request.desired_frame = "base_footprint"; // better than base_link
    if (!collision_processing_srv.call(processing_call))
    {
        ROS_ERROR("Collision map processing service failed");
        return -1;
    }
    // The collision map processor returns instances of graspable objects
    if (processing_call.response.graspable_objects.empty())
    {
        ROS_ERROR("Collision map processing returned no graspable objects");
        return -1;
    }
    else
    {
        ROS_WARN("Collision map processing succeeded and returned %d graspable objects", processing_call.response.graspable_objects.size() );
    }


    // Get the table height, which we use as the z value for the nearest object location
    double table_height;
    if (getTableHeight(detection_call.response.detection.table.pose, table_height))
    {
        ROS_INFO("Table height = %f ", table_height );
    }


    // Define optimal pickup location, from where nearest object will be found
    geometry_msgs::PointStamped pickup_point;
    pickup_point.header.frame_id = "/base_link"; // REEM base_link x position is level with front of chest
    pickup_point.point.x = 0.15; // 15cm in front of REEM's chest,
    pickup_point.point.y = 0.20; // 20cm to left, for left arm
    pickup_point.point.z = table_height;


    // Which is the nearest object returned by the collision map processor, based on our optimal pickup location
    int object_to_pick_ind;
    if (getNearestObjectIndex(processing_call.response.graspable_objects, pickup_point, object_to_pick_ind))
    {
        ROS_INFO("Nearest object ID = %d ", object_to_pick_ind );
    }
    else
    {
        ROS_ERROR("Could not calculate nearest object");
        return -1;
    }


    // Sync the Planning Scene
    // This will publish planning_scene_markers for Rviz, showing the table and object markers
    arm_navigation_msgs::GetPlanningScene::Request planning_scene_req;
    arm_navigation_msgs::GetPlanningScene::Response planning_scene_res;
    if(!get_planning_scene_client.call(planning_scene_req, planning_scene_res))
    {
        ROS_WARN("Can't get planning scene");
        return -1;
    }


} // end of test


// publishes a TF marker frame /reference_point
bool getNearestObjectIndex(std::vector<object_manipulation_msgs::GraspableObject>& objects, geometry_msgs::PointStamped& reference_point, int& object_ind)
{
    // Convert reference point to base_link frame
    geometry_msgs::PointStamped point;
    tf_listener->transformPoint("/base_link", reference_point, point);

    // Publish TF marker for optimal pickup location
    tf::StampedTransform tf_transform;
    tf_transform.stamp_ = ros::Time::now()+ros::Duration(0.5);  // future publish by 0.5 seconds
    tf_transform.setOrigin( tf::Vector3(reference_point.point.x, reference_point.point.y, reference_point.point.z ) );
    tf_transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
    tf_transform.frame_id_ = reference_point.header.frame_id; // same frame_id as reference_point e.g. base_footprint
    tf_transform.child_frame_id_ = "reference_point";
    tf_broadcaster->sendTransform(tf_transform);

    // find the closest object
    double nearest_dist = 1e6;
    int nearest_object_ind = -1;
    for (size_t i = 0; i < objects.size(); ++i)
    {
        sensor_msgs::PointCloud cloud;
        tf_listener->transformPointCloud("/base_link", objects[i].cluster, cloud);

        // calculate average
        float x = 0.0, y = 0.0, z = 0.0;
        for (size_t j = 0; j < cloud.points.size(); ++j)
        {
            x += cloud.points[j].x;
            y += cloud.points[j].y;
            z += cloud.points[j].z;
        }
        x /= cloud.points.size();
        y /= cloud.points.size();
        z /= cloud.points.size();

        double dist = sqrt(pow(x - point.point.x, 2.0) + pow(y - point.point.y, 2.0) + pow(z - point.point.z, 2.0));
        if (dist < nearest_dist)
        {
            nearest_dist = dist;
            nearest_object_ind = i;
        }
    }

    if (nearest_object_ind > -1)
    {
        ROS_INFO("nearest object ind: %d (distance: %f)", nearest_object_ind, nearest_dist);
        object_ind = nearest_object_ind;
        return true;
    }
    else
    {
        ROS_ERROR("No nearby objects. Unable to select grasp target");
        return false;
    }
}


// publishes a TF marker frame /table_origin
bool getTableHeight(geometry_msgs::PoseStamped& _table_pose, double& table_height)
//std::vector<object_manipulation_msgs::GraspableObject>& objects, geometry_msgs::PointStamped& reference_point, int& object_ind)
{
    // Display TF marker at origin of detected table
    tf::StampedTransform tf_transform;
    tf_transform.stamp_ = ros::Time::now()+ros::Duration(0.5);  // future publish by 0.5 seconds
    tf::Stamped<tf::Pose> tf_pose;
    tf::poseStampedMsgToTF(_table_pose, tf_pose); // convert table PoseStamped into a TF transform
    tf_transform.setRotation( tf_pose.getRotation() );
    tf_transform.setOrigin( tf_pose.getOrigin() );
    tf_transform.frame_id_ = "/base_link";
    tf_transform.child_frame_id_ = "table_origin";
    tf_broadcaster->sendTransform(tf_transform);

    // Convert origin of detected table to base_footprint frame
    geometry_msgs::PointStamped table_origin_point;
    geometry_msgs::PoseStamped table_origin;
    tf_listener->transformPose("/base_footprint", _table_pose, table_origin);

    ROS_INFO("table_origin_point:  x: %f  y: %f  z: %f ", table_origin.pose.position.x, table_origin.pose.position.y, table_origin.pose.position.z );
    ROS_INFO("Table height = %f ", table_origin.pose.position.z );

    table_height = table_origin.pose.position.z; // vertical height from base_link
    return true;
}

