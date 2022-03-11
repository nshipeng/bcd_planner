/*
 * @Description: 
 * @Autor: 
 */
// /*
//  * @Description: 
//  * @Autor: 
//  */
#include "bcd_planner_wrapper.h"
#include "bcd_planner.h"


bcd_planner_wrapper::bcd_planner_wrapper(){
     bcd_planner_ptr_ = std::make_shared<bcd_planner>();
}


void bcd_planner_wrapper::plan(std::vector<std::vector<Vector2d>>& wall_polygon, std::vector<std::vector<Vector2d>>&obstcal_polygon, double robot_radius,std::deque<Vector2d>& path){
    bcd_planner_ptr_ ->plan(wall_polygon,obstcal_polygon,robot_radius,path);
}
void bcd_planner_wrapper::plan(std::vector<std::vector<Vector3d>>& wall_polygon, std::vector<std::vector<Vector3d>>&obstcal_polygon, double robot_radius,std::deque<Vector2d>& path){
    bcd_planner_ptr_ ->plan(wall_polygon,obstcal_polygon,robot_radius,path);
}