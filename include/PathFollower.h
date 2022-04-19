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

/* user includes */

//}

namespace path_follower {

/* class PathFollower //{ */
    class PathFollower : public nodelet::Nodelet {

    public:
        /* onInit() is called when nodelet is launched (similar to main() in regular node) */
        virtual void onInit();

    private:
        /* flags */
        bool m_is_initialized = false;

        /* ros parameters */
        std::string m_uav_name;

        /* other parameters */
        // TODO: consider using std::list here for better performance and code simplicity
        std::vector<mrs_msgs::Reference> m_points_to_follow;
        std::string m_frame_of_reference;
        size_t m_current_point_to_follow_index;
        int m_sequence_counter = 0;
        int m_points_to_load_at_once = 5;
        double m_threshold_distance_to_first_point = 1.0;
        bool m_first_point_passed = false;

        bool m_new_path_request = false;
        mrs_msgs::Path m_path_message_template;

        // | --------------------- MRS transformer -------------------- |

        mrs_lib::Transformer m_transformer;

        ros::ServiceClient m_trajectory_generator_service_client;

        void update_path_message_template(const mrs_msgs::PathSrv::Request &req);
        ros::ServiceServer m_service_server_follow_path;
        bool callback_follow_path_srv(mrs_msgs::PathSrv::Request &req, mrs_msgs::PathSrv::Response &res);

        void send_new_trajectory_chunk(size_t n);
        ros::Subscriber m_subscriber_slow_odom;
        void callback_slow_odom(const nav_msgs::Odometry &msg);

        mrs_msgs::Path _generate_path_for_simulation_one_drone(std::vector<std::pair<double, double>> &points_to_visit);

    };
//}

}  // namespace path_follower
