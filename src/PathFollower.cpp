#include <PathFollower.h>

/* every nodelet must include macros which export the class as a nodelet plugin */
#include <pluginlib/class_list_macros.h>
#include <mrs_msgs/PathSrv.h>
#include "utils.h"
#include <std_srvs/Trigger.h>

namespace path_follower {

/* onInit() method //{ */
    void PathFollower::onInit() {

        // | ---------------- set my booleans to false ---------------- |
        // but remember, always set them to their default value in the header file
        // because, when you add new one later, you might forget to come back here

        /* obtain node handle */
        ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();

        /* waits for the ROS to publish clock */
        ros::Time::waitForValid();

        // | ------------------- load ros parameters ------------------ |
        /* (mrs_lib implementation checks whether the parameter was loaded or not) */
        mrs_lib::ParamLoader pl(nh, "PathFollower");

        pl.loadParam("UAV_NAME", m_uav_name);

        if (!pl.loadedSuccessfully()) {
            ROS_ERROR("[PathFollower]: failed to load non-optional parameters!");
            ros::shutdown();
        } else {
            ROS_INFO_ONCE("[PathFollower]: loaded parameters");
        }

        // | --------------------- tf transformer --------------------- |
        m_transformer = mrs_lib::Transformer("PathFollower");
        m_transformer.setDefaultPrefix(m_uav_name);

        m_trajectory_generator_service_client = nh.serviceClient<mrs_msgs::PathSrv>(
                "/" + m_uav_name + "/trajectory_generation/path");
        if (!m_trajectory_generator_service_client.isValid()) {
            ROS_WARN("[PathFollower]: Path generation service is not valid");
        }

        m_control_manager_stop_following_service_client = nh.serviceClient<std_srvs::Trigger>("/" + m_uav_name + "/control_manager/stop_trajectory_tracking");
        if (!m_control_manager_stop_following_service_client.isValid()) {
            ROS_WARN("[PathFollower]: Could not connect to control_manager/stop_trajectory_tracking service");
        }

        m_control_manager_start_following_service_client = nh.serviceClient<std_srvs::Trigger>("/" + m_uav_name + "/control_manager/start_trajectory_tracking");
        if (!m_control_manager_start_following_service_client.isValid()) {
            ROS_WARN("[PathFollower]: Could not connect to control_manager/start_trajectory_tracking service");
        }

        m_service_server_follow_path = nh.advertiseService("/" + m_uav_name + "/path_to_follow", &PathFollower::callback_follow_path_srv,
                                                           this);


        ROS_INFO_ONCE("[PathFollower]: initialized");

        m_is_initialized = true;

    }


    void PathFollower::update_path_message_template(const mrs_msgs::PathSrv::Request &req) {
        // Copy some parameters to remember the user preferences. Copy element-wise to not copy all the points too
        m_path_message_template.loop = false;
        m_path_message_template.fly_now = true;
        m_path_message_template.override_constraints = req.path.override_constraints;
        m_path_message_template.relax_heading = req.path.relax_heading;
        m_path_message_template.use_heading = req.path.use_heading;
        m_path_message_template.stop_at_waypoints = req.path.stop_at_waypoints;
        m_path_message_template.override_max_acceleration_horizontal = req.path.override_max_acceleration_horizontal;
        m_path_message_template.override_max_acceleration_vertical = req.path.override_max_acceleration_vertical;
        m_path_message_template.override_max_jerk_horizontal = req.path.override_max_jerk_horizontal;
        m_path_message_template.override_max_jerk_vertical = req.path.override_max_jerk_vertical;
        m_path_message_template.override_max_velocity_horizontal = req.path.override_max_velocity_horizontal;
        m_path_message_template.override_max_velocity_vertical = req.path.override_max_velocity_vertical;
    }

    bool PathFollower::callback_follow_path_srv(mrs_msgs::PathSrv::Request &req, mrs_msgs::PathSrv::Response &res) {
        res.success = false;
        if (req.path.fly_now == false || req.path.loop == true) {
            res.message = "Error. False value of fly_now and true value of loop are not supported still...";
            return true;
        }
        if (req.path.points.empty()) {
            res.message = "Error: empty path";
            return true;
        }
        // TODO: make frame a configurable parameter, and not just hardcode it
        auto transform = m_transformer.getTransform(req.path.header.frame_id, "gps_origin");
        if (!transform.has_value()) {
            ROS_ERROR("[PathFollower]: ERROR: could not get transform to local origin");
            res.message = "ERROR: could not get transform to local origin";
            return true;
        }

        // Transform all the points to gps_origin and save the altitude
        auto path_transformed = req.path.points;
        for (auto &p: path_transformed) {
            mrs_msgs::ReferenceStamped ref;
            ref.reference = p;
            ref.header.frame_id = req.path.header.frame_id;

            geometry_msgs::Point point = p.position;
            double altitude = point.z;

            auto points_transformed = m_transformer.transform(point, transform.value());
            if (!points_transformed.has_value()) {
                ROS_ERROR("[PathFollower]: Error. Could not transform path");
                res.message = "Error. Could not transform path";
                return true;
            }

            p.position = points_transformed.value();
            if (req.path.header.frame_id == "latlon_origin") {
                p.position.z = altitude;
            }
        }
        // Update message header, containing constraints, speed configuration...
        update_path_message_template(req);
        req.path.header.seq = m_sequence_counter++;
        req.path.header.frame_id = "gps_origin";
        req.path.points = path_transformed;
        add_heading_to_path(req.path);

        // Try to make the drone stop for trajectory regeneration. Don't check the result as even if the drone is not stopped -
        // trajectory may be regenerated
        std_srvs::Trigger trigger;
        m_control_manager_stop_following_service_client.call(trigger);
        // Sleep for some time to let the drone stop
        auto wait_time = ros::Duration(2, 0);
        wait_time.sleep();

        req.path.fly_now = false;
        m_trajectory_generator_service_client.call(req, res);
        m_points_to_follow.clear();
        if (!res.success) {
            ROS_ERROR_STREAM("[PathFollower]: Error while calling trajectory generation. Message: " << res.message);
        } else {
            std_srvs::Trigger trigger_start_tracking;
            if (!m_control_manager_start_following_service_client.call(trigger_start_tracking) || !trigger_start_tracking.response.success) {
                ROS_ERROR_STREAM("[PathFollower]: Could not start trajectory tracking. Message: " << trigger_start_tracking.response.message);
                res.message = "Could not call service to start the trajectory tracking";
            }
        }
        return true;
    }

    void PathFollower::add_heading_to_path(mrs_msgs::Path &path) {
        bool reverse = false;
        double old_heading = 0;
        bool first_not_straight = true;
        //TODO: move this staff to path_generator and combine them somehow
        for (size_t i = 1; i + 2 < path.points.size(); ++i) {
            auto angle1 = std::atan2(path.points[i].position.y - path.points[i - 1].position.y,
                               path.points[i].position.x - path.points[i - 1].position.x);
            auto angle2 = std::atan2(path.points[i + 1].position.y - path.points[i].position.y,
                                     path.points[i + 1].position.x - path.points[i].position.x);

            auto angle3 = std::atan2(path.points[i + 2].position.y - path.points[i + 1].position.y,
                                    path.points[i + 2].position.x - path.points[i + 1].position.x);

            if (std::abs(angle2 - angle1) <= 1e-2) {
                old_heading = path.points[i].heading = angle1 + (reverse ? M_PI : 0);
                first_not_straight = true;
            } else if (std::abs(angle2 - angle3) <= 1e-2) {
                old_heading = path.points[i].heading = angle2 + (reverse ? M_PI : 0);
                first_not_straight = true;
            } else {
                if (first_not_straight) {
                    first_not_straight = false;
                    reverse = !reverse;
                }

            }
            path.points[i].heading = old_heading;
        }
    }
//}

}  // namespace path_follower  

/* every nodelet must export its class as nodelet plugin */
PLUGINLIB_EXPORT_CLASS(path_follower::PathFollower, nodelet::Nodelet)
