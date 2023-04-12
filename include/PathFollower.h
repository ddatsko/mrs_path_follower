#pragma once
/* includes //{ */

/* each ROS nodelet must have these */
#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

/* some STL includes */
#include <cstdlib>
#include <cstdio>

/* custom helper functions from our library */
#include <mrs_lib/param_loader.h>
#include <mrs_lib/transformer.h>

/* other important includes */
#include <nav_msgs/Odometry.h>
#include <mrs_msgs/PathSrv.h>
#include <std_srvs/Trigger.h>

/* user includes */

#include <mrs_msgs/DynamicsConstraints.h>

namespace path_follower {

/* class PathFollower //{ */
    class PathFollower : public nodelet::Nodelet {

    public:
        /* onInit() is called when nodelet is launched (similar to main() in regular node) */
        virtual void onInit();

    private:
        /* flags */
        bool m_is_initialized = false;
        bool m_constraints_loaded = false;

        /* ros parameters */
        std::string m_uav_name;

        /* other parameters */
        // TODO: consider using std::list here for better performance and code simplicity
        std::string m_frame_of_reference;
        size_t m_current_point_to_follow_index;
        int m_sequence_counter = 0;
        double m_visited_point_tolerance = 0.5;
        bool m_is_in_pause = false;
        mrs_msgs::PathSrv::Request m_last_request;

        bool m_new_path_request = false;
        mrs_msgs::Path m_path_message_template;
        mrs_msgs::DynamicsConstraints m_current_constraints;

        // | --------------------- MRS transformer -------------------- |

        mrs_lib::Transformer m_transformer;

        ros::ServiceClient m_trajectory_generator_service_client;
        ros::ServiceClient m_control_manager_stop_following_service_client;
        ros::ServiceClient m_control_manager_start_following_service_client;

        ros::ServiceServer m_service_server_follow_path;
        ros::ServiceServer m_service_server_pause;
        ros::ServiceServer m_service_server_resume;

        ros::Subscriber m_current_constraints_subscriber;
        ros::Subscriber m_odometry_subscriber;

        void current_constraints_callback(const mrs_msgs::DynamicsConstraints &constraints);
        void odometry_callback(const nav_msgs::Odometry &odometry);

        void update_path_message_template(const mrs_msgs::PathSrv::Request &req);

        bool callback_follow_path_srv(mrs_msgs::PathSrv::Request &req, mrs_msgs::PathSrv::Response &res);
        bool callback_pause(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
        bool callback_resume(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);




    };
//}

}  // namespace path_follower
