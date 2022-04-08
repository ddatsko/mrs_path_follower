//
// Created by mrs on 08.04.22.
//

#include "utils.h"
#include <cmath>


double distance_between_points(const geometry_msgs::Point &p1, const geometry_msgs::Point &p2) {
    return std::sqrt(std::pow(p2.x - p1.x, 2) +
                        std::pow(p2.y - p1.y, 2) +
                        std::pow(p2.z - p1.z, 2));
}