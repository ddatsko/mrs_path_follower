#include <PathFollower.h>

/* every nodelet must include macros which export the class as a nodelet plugin */
#include <pluginlib/class_list_macros.h>
#include <mrs_msgs/PathSrv.h>
#include "utils.h"

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
        m_transformer = mrs_lib::Transformer("PathFollower", m_uav_name);

        m_trajectory_generator_service_client = nh.serviceClient<mrs_msgs::PathSrv>(
                "/" + m_uav_name + "/trajectory_generation/path");
        if (!m_trajectory_generator_service_client.isValid()) {
            ROS_ERROR("Path generation service is not valid");
            ros::shutdown();
            return;
        }
        m_service_server_follow_path = nh.advertiseService("/" + m_uav_name + "/path_to_follow", &PathFollower::callback_follow_path_srv,
                                                           this);
        m_subscriber_slow_odom = nh.subscribe("/" + m_uav_name + "/odometry/slow_odom", 1,
                                              &PathFollower::callback_slow_odom, this,
                                              ros::TransportHints().tcpNoDelay());


        ROS_INFO_ONCE("[PathFollower]: initialized");

        m_is_initialized = true;

    }


    mrs_msgs::Path
    PathFollower::_generate_path_for_simulation_one_drone(std::vector<std::pair<double, double>> &points_to_visit) {
        mrs_msgs::Path path;

        // Set the parameters for trajectory generation
        path.header.stamp = ros::Time::now();
        path.header.seq = m_sequence_counter++;
        path.header.frame_id = "gps_origin";

        path.fly_now = true;
        path.use_heading = false;
        path.stop_at_waypoints = false;
        path.loop = false;
        path.override_constraints = false;

        // TODO: find out what this parameter means
        path.relax_heading = true;

        std::vector<mrs_msgs::Reference> points;

        for (const auto &p: points_to_visit) {
            mrs_msgs::Reference point_3d;
            point_3d.heading = 1;

            point_3d.position.x = p.first;
            point_3d.position.y = p.second;
            point_3d.position.z = m_drones_altitude;
            points.push_back(point_3d);
        }
        path.points = points;
        return path;

    }

    void PathFollower::callback_slow_odom(const nav_msgs::Odometry &msg) {
        if (m_points_to_follow.empty()) {
            return;
        }
        if (m_current_point_to_follow_index >= m_points_to_follow.size()) {
            ROS_INFO ("Path following finished");
            m_points_to_follow.clear();
            m_current_point_to_follow_index = 0;
            return;
        }
        if (m_new_path_request) {
            m_new_path_request = false;
            m_current_point_to_follow_index = 0;
            send_new_trajectory_chunk(m_points_to_load_at_once);
        } else {
            auto transform = m_transformer.getTransform(msg.header.frame_id, "gps_origin");
            if (!transform.has_value()) {
                ROS_ERROR("Could not get the transform to transform odometry message to gps");
                return;
            }
            auto gps_frame_msg = m_transformer.transform(transform.value(), msg);
            if (!gps_frame_msg.has_value()) {
                ROS_ERROR("Could not transform odometry message to gps frame");
                return;
            }
            const auto &current_position = gps_frame_msg.value().pose.pose.position;
            // If the UAV is heading towards the start point and the distance is small enough - load first chung of points
            if (distance_between_points(current_position, m_points_to_follow[m_current_point_to_follow_index].position) <=
                m_threshold_distance_to_first_point) {

                m_first_point_passed = true;
            }
            // If the first point is already passed and the drone crossed the halfway to the next point = recalculate its trajectory with one new point
            if (m_first_point_passed && m_current_point_to_follow_index + 1 < m_points_to_follow.size() &&
                distance_between_points(current_position, m_points_to_follow[m_current_point_to_follow_index].position) >
                1) {

                m_current_point_to_follow_index++;
                send_new_trajectory_chunk(m_points_to_load_at_once);
                m_first_point_passed = false;
            }
        }

    }


    void PathFollower::update_path_message_template(const mrs_msgs::PathSrv::Request &req) {
        // Copy some parameters to remember the user preferences. Copy element-wise to not copy all the points too
        m_path_message_template.fly_now = true;
        m_path_message_template.loop = false;
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
        if (req.path.fly_now == false || req.path.loop == true) {
            res.success = false;
            res.message = "Error. False value of fly_now and true value of loop are not supported still...";
            return true;
        }
        update_path_message_template(req);
        m_points_to_follow = req.path.points;
        m_new_path_request = true;
        res.success = true;
        return true;
    }

    void PathFollower::send_new_trajectory_chunk(size_t n) {
        mrs_msgs::PathSrv msg;
        msg.request.path = m_path_message_template;
        msg.request.path.points.reserve(n);
        size_t i;
        for (i = m_current_point_to_follow_index;
             i < m_current_point_to_follow_index + n && i < m_points_to_follow.size(); ++i) {
            msg.request.path.points.push_back(m_points_to_follow[i]);
        }

        ROS_INFO_STREAM("Sending new path to the planner. Path: \n");
        for (const auto &p: msg.request.path.points) {
            ROS_INFO_STREAM(p.position.x << " " << p.position.y << " " << p.position.z << ", Heading: " << p.heading << "\n");
        }

        bool call_res = m_trajectory_generator_service_client.call(msg);
        if (!call_res) {
            ROS_ERROR_STREAM("Could not connect to trajectory generation service. Message: " << msg.response.message);
        } else {
            ROS_INFO_STREAM("Successfully loaded new path of size " << i);
        }
    }

//}

}  // namespace path_follower  

/* every nodelet must export its class as nodelet plugin */
PLUGINLIB_EXPORT_CLASS(path_follower::PathFollower, nodelet::Nodelet)
