/*
 * @Description: 
 * @Autor: 
 */
#include <memory>
#include "bcd_planner.h"

int main(int argc, char** argv){

    std::shared_ptr<bcd_planner> bcd_planner_ptr_ = std::make_shared<bcd_planner>();
    bcd_planner_ptr_ ->TestAllExamples();

    // {
    //     std::vector<cv::Point> handcrafted_polygon_2 = {cv::Point(100,500), cv::Point(200,500), cv::Point(200,400), cv::Point(400,400),
    //                                                 cv::Point(400,500), cv::Point(500,500), cv::Point(500,100), cv::Point(400,100),
    //                                                 cv::Point(400,200), cv::Point(200,200), cv::Point(200,100), cv::Point(100,100),
    //                                                 };
    //     std::vector<std::vector<cv::Point>> contours = {handcrafted_polygon_2};
    //     std::shared_ptr<bcd_planner> bcd_planner_ptr_ = std::make_shared<bcd_planner>();

    // }
    

    return 0;
}