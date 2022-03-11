/*
 * @Description: 
 * @Autor: 
 */
#ifndef _BCD_PLANNER_WRAPPER_H_
#define _BCD_PLANNER_WRAPPER_H_

#include <memory>
#include <vector>
#include <deque>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

class bcd_planner;

using namespace Eigen;
class bcd_planner_wrapper
{

    public:
       bcd_planner_wrapper();

       void plan(std::vector<std::vector<Vector2d>>& wall_polygon, std::vector<std::vector<Vector2d>>&obstcal_polygon, double robot_radius,std::deque<Vector2d>& path);
       void plan(std::vector<std::vector<Vector3d>>& wall_polygon, std::vector<std::vector<Vector3d>>&obstcal_polygon, double robot_radius,std::deque<Vector2d>& path);


    private:
        std::shared_ptr<bcd_planner> bcd_planner_ptr_;
};















#endif