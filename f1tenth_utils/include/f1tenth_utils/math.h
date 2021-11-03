#ifndef F1TENTH_UTILS_MATH_H
#define F1TENTH_UTILS_MATH_H

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>

double calculateDistanceSq(const double x1, const double y1, const double x2, const double y2);

double calculateDistance(const double x1, const double y1, const double x2, const double y2);

/**
 * @brief Calculates the arc curvature joining a pose and point which are defined in a common reference frame.
 *
 * @sa https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf.
 * @param pose Pose of the arc origin.
 * @param point End point of the arc.
 * @return Arc curvature.
 */
double calculateCurvature(const geometry_msgs::Pose& pose, const geometry_msgs::Point point);

#endif  // F1TENTH_UTILS_MATH_H
