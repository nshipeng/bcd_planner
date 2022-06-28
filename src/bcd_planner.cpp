/*
 * @Description: 
 * @Autor: 
 */

#include "bcd_planner.h"

bcd_planner::bcd_planner(){
    init();
}


void bcd_planner::plan(std::vector<std::vector<cv::Point>>& wall_polygon, std::vector<std::vector<cv::Point>>&obstcal_polygon, double robot_radius,double step,std::deque<Point2D>& path){
    step *= inv_resolution;
    plan_in(wall_polygon,obstcal_polygon,robot_radius,step,path);
}

void bcd_planner::plan(std::vector<std::vector<Vector2d>>& wall_polygon, std::vector<std::vector<Vector2d>>&obstcal_polygon, double robot_radius,double step,std::deque<Vector2d>& path){
     std::vector<std::vector<cv::Point>>wall_polygon_points;
     for(int i = 0;i<wall_polygon.size(); i++){
         scal_polygon(wall_polygon[i],robot_radius);           //scale polygon in {world} coordinate.
         std::vector<cv::Point> points;
         for(int j = 0; j<wall_polygon[i].size();j++){
             Vector2i map_point = world2map(wall_polygon[i][j](0),wall_polygon[i][j](1));
             points.push_back(cv::Point(map_point(0),map_point(1)));
         }
         wall_polygon_points.push_back(points);
     }

     std::vector<std::vector<cv::Point>>obstacl_polygon_points;
     for(int i = 0;i<obstcal_polygon.size(); i++){
         std::vector<cv::Point> obstacl_points;
         for(int j = 0; j<obstcal_polygon[i].size();j++){
             Vector2i map_point = world2map(obstcal_polygon[i][j](0),obstcal_polygon[i][j](1));
             obstacl_points.push_back(cv::Point(map_point(0),map_point(1)));
         }
         obstacl_polygon_points.push_back(obstacl_points);
     }

     std::deque<Point2D>deque_path;
     step *= inv_resolution;
     plan_in(wall_polygon_points,obstacl_polygon_points,robot_radius,step,deque_path);
     for(auto p : deque_path){
         Vector2d path_point = map2world(p.x,p.y);
         path.push_back(Vector2d(path_point(0),path_point(1)));
     }
 }

void bcd_planner::plan(std::vector<std::vector<Vector3d>>& wall_polygon, std::vector<std::vector<Vector3d>>&obstcal_polygon, double robot_radius,double step,std::deque<Vector2d>& path){
    std::vector<std::vector<Vector2d>>  wall_polygon_2d;
    for(int i = 0;i<wall_polygon.size(); i++){
        std::vector<Vector2d> points;
        for(int j = 0; j<wall_polygon[i].size();j++){
             points.push_back(Vector2d(wall_polygon[i][j](0),wall_polygon[i][j](1)));
         }
         wall_polygon_2d.push_back(points);
     }

    std::vector<std::vector<Vector2d>>  obstcal_polygon_2d;
    for(int i = 0;i<obstcal_polygon.size(); i++){
        std::vector<Vector2d>  obstacl_points;
         for(int j = 0; j<obstcal_polygon[i].size();j++){
             obstacl_points.push_back(Vector2d(obstcal_polygon[i][j](0),obstcal_polygon[i][j](1)));
         }
         obstcal_polygon_2d.push_back(obstacl_points);
     }
     plan( wall_polygon_2d,obstcal_polygon_2d,robot_radius,step,path);
}

void bcd_planner::inflation(cv::Mat& map,double robot_radius){
    std::cout<<"inflate map"<<std::endl;

    cv::Mat map_copy = map;

    //int inflate_size = robot_radius / resolution;
    //std::cout<<"inflate_size  "<<inflate_size <<std::endl;
    int inflate_size = 5;
    for(int i = 0;i<num_of_colums;i++){
        for(int j = 0; j< num_of_rows; j++){
            if(i ==0 || j ==0){
                continue;
            }
            if(map.at<cv::Vec3b>(j, i) == cv::Vec3b(0,0,0)){
                std::cout<<"i "<<i<<" j "<<j<<std::endl;
                for(int dx = -inflate_size; dx<= inflate_size; dx++){
                    for(int dy = -inflate_size; dy<=inflate_size; dy++){
                        if((i+dx) >= num_of_colums || (i+dx) <= 0 || (j+dy)>= num_of_rows || (j+dy)<=0){
                            continue;
                        }
                        if((dx * dx + dy* dy) >(inflate_size*inflate_size)){
                            continue;
                        }
                        map_copy.at<cv::Vec3b>(j+dy, i+dx) = cv::Vec3b(0,0,0);
                    }
                }
            }
        }
    }
    map = map_copy;

}


void bcd_planner::plan_in(std::vector<std::vector<cv::Point>>& wall_polygon, std::vector<std::vector<cv::Point>>&obstcal_polygon, double robot_radius, double step,std::deque<Point2D>& path){

    cv::Mat1b map = cv::Mat1b(cv::Size(600, 600), CV_8U);        //初始化map
    map.setTo(0);
    
    cv::fillPoly(map, wall_polygon, 255);                         //用墙进行隔断
    cv::fillPoly(map, obstcal_polygon , 0);                       //polygon内部的障碍物


    //inflation(map,robot_radius);

    std::vector<std::vector<cv::Point>> obstacle_contours;
    std::vector<std::vector<cv::Point>> wall_contours;
    ExtractContours(map, wall_contours, obstacle_contours);

    PolygonList obstacles = ConstructObstacles(map, obstacle_contours);
    Polygon wall = ConstructWall(map, wall_contours.front());

    cv::Mat3b map_ = cv::Mat3b(map.size());
    map_.setTo(cv::Scalar(0, 0, 0));

    cv::fillPoly(map_, wall_contours, cv::Scalar(255, 255, 255));        
    cv::fillPoly(map_, obstacle_contours, cv::Scalar(0, 0, 0));

    
    std::vector<Event> wall_event_list = GenerateWallEventList(map_, wall);                              //根据wall polygon中的每个点的event类型来生成wall_event_list                                                       
    std::vector<Event> obstacle_event_list = GenerateObstacleEventList(map_, obstacles);                 //根据obstacle_event_list中的每个点的event类型来生成obstac_event_list
    std::deque<std::deque<Event>> slice_list = SliceListGenerator(wall_event_list, obstacle_event_list);
    if(show_map){
        CheckSlicelist(slice_list);
    }
    
    std::vector<CellNode> cell_graph;
    std::vector<int> cell_index_slice;
    std::vector<int> original_cell_index_slice;
    ExecuteCellDecomposition(cell_graph, cell_index_slice, original_cell_index_slice, slice_list);

    if(show_map){
        CheckPointType(map, wall ,obstacles);
        CheckGeneratedCells(map, cell_graph);
    }
  
    Point2D start = cell_graph.front().ceiling.front();
    std::deque<std::deque<Point2D>> original_planning_path = StaticPathPlanning(map, cell_graph, start, step, false, false);
    if(show_map){
        CheckPathNodes(original_planning_path);
    }
   
    path = FilterTrajectory(original_planning_path);
    if(show_map){
        CheckPathConsistency(path);
        int time_interval = 1;
        VisualizeTrajectory(map, path, robot_radius, PATH_MODE, time_interval);
    }
   
    
 }

int bcd_planner::WrappedIndex(int index, int list_length){
    int wrapped_index = (index%list_length+list_length)%list_length;
    return wrapped_index;
}


void bcd_planner::WalkThroughGraph(std::vector<CellNode>& cell_graph, int cell_index, int& unvisited_counter, std::deque<CellNode>& path){
        if(!cell_graph[cell_index].isVisited)
    {
        cell_graph[cell_index].isVisited = true;
        unvisited_counter--;
    }
    path.emplace_front(cell_graph[cell_index]);

    CellNode neighbor;
    int neighbor_idx = INT_MAX;

    for(int i = 0; i < cell_graph[cell_index].neighbor_indices.size(); i++)
    {
        neighbor = cell_graph[cell_graph[cell_index].neighbor_indices[i]];
        neighbor_idx = cell_graph[cell_index].neighbor_indices[i];
        if(!neighbor.isVisited)
        {
            break;
        }
    }

    if(!neighbor.isVisited) // unvisited neighbor found
    {
        cell_graph[neighbor_idx].parentIndex = cell_graph[cell_index].cellIndex;
        WalkThroughGraph(cell_graph, neighbor_idx, unvisited_counter, path);
    }
    else  // unvisited neighbor not found
    {

        if (cell_graph[cell_index].parentIndex == INT_MAX) // cannot go on back-tracking
        {
            return;
        }
        else if(unvisited_counter == 0)
        {
            return;
        }
        else
        {
            WalkThroughGraph(cell_graph, cell_graph[cell_index].parentIndex, unvisited_counter, path);
        }
    }
}

 std::deque<CellNode> bcd_planner::GetVisittingPath(std::vector<CellNode>& cell_graph, int first_cell_index){
    std::deque<CellNode> visitting_path;
    if(cell_graph.size()==1)
    {
        visitting_path.emplace_back(cell_graph.front());
    }
    else
    {
        int unvisited_counter = cell_graph.size();
        WalkThroughGraph(cell_graph, first_cell_index, unvisited_counter, visitting_path);
        std::reverse(visitting_path.begin(), visitting_path.end());
    }

    return visitting_path;
 }

 std::vector<Point2D> bcd_planner::ComputeCellCornerPoints(const CellNode& cell){
    Point2D topleft = cell.ceiling.front();
    Point2D bottomleft = cell.floor.front();
    Point2D bottomright = cell.floor.back();
    Point2D topright = cell.ceiling.back();                                           

    std::vector<Point2D> corner_points = {topleft, bottomleft, bottomright, topright};
    return corner_points;
 }

 std::vector<int> bcd_planner::DetermineCellIndex(std::vector<CellNode>& cell_graph, const Point2D& point){
    std::vector<int> cell_index;
    for(int i = 0; i < cell_graph.size(); i++)
    {
        for(int j = 0; j < cell_graph[i].ceiling.size(); j++)
        {
            if(point.x ==  cell_graph[i].ceiling[j].x && point.y >= cell_graph[i].ceiling[j].y && point.y <= cell_graph[i].floor[j].y) 
            {
                cell_index.emplace_back(int(i));
            }
        }

    }
    return cell_index;
 }

std::deque<Point2D> bcd_planner::GetBoustrophedonPath(std::vector<CellNode>& cell_graph, CellNode cell, int corner_indicator, int robot_radius){
    int delta, increment;

    std::deque<Point2D> path;

    std::vector<Point2D> corner_points = ComputeCellCornerPoints(cell);

    std::vector<Point2D> ceiling, floor;
    ceiling.assign(cell.ceiling.begin(), cell.ceiling.end());
    floor.assign(cell.floor.begin(), cell.floor.end());

    if(cell_graph[cell.cellIndex].isCleaned)    
    {
        if(corner_indicator == TOPLEFT)
        {
            path.emplace_back(corner_points[TOPLEFT]);
        }
        if(corner_indicator == TOPRIGHT)
        {
            path.emplace_back(corner_points[TOPRIGHT]);
        }
        if(corner_indicator == BOTTOMLEFT)
        {
            path.emplace_back(corner_points[BOTTOMLEFT]);
        }
        if(corner_indicator == BOTTOMRIGHT)
        {
            path.emplace_back(corner_points[BOTTOMRIGHT]);
        }
    }
    else
    {
        if(corner_indicator == TOPLEFT)
        {
            int x, y, y_start, y_end;
            bool reverse = false;

            for(int i = 0; i < ceiling.size(); i = i + (robot_radius+1))
            {
                x = ceiling[i].x;

                if(!reverse)
                {
                    y_start = ceiling[i].y;
                    y_end   = floor[i].y;

                    for(y = y_start; y <= y_end; y++)
                    {
                        path.emplace_back(Point2D(x, y));    //向往下走到底
                    }

                    if((std::abs(floor[i+1].y-floor[i].y)>=2)&&(i+1<floor.size()))  
                    {
                        delta = floor[i+1].y-floor[i].y;
                        increment = delta/abs(delta);
                        for(int k = 1; k <= abs(delta); k++)
                        {
                            path.emplace_back(Point2D(floor[i].x, floor[i].y + increment * (k)));  //??
                        }
                    }

                    if(robot_radius != 0)
                    {
                        for(int j = 1; j <= robot_radius+1; j++)
                        {
                            // 沿着floor从左往右
                            if( x+j >= floor.back().x)
                            {
                                i = i - (robot_radius - (j - 1));
                                break;
                            }

                            //提前转
                            else if((floor[i+(j)].y-floor[i+(j+1)].y>=2)
                               &&(j<=robot_radius+1)
                               &&(j+1<=robot_radius+1))
                            {
                                delta = floor[i+(j+1)].y-floor[i+(j)].y;
                                increment = delta/abs(delta);
                                for(int k = 0; k <= abs(delta); k++)
                                {
                                    path.emplace_back(Point2D(floor[i+(j)].x, floor[i+(j)].y+increment*(k)));
                                }
                            }
                            //滞后转
                            else if((floor[i+(j+1)].y-floor[i+(j)].y>=2)
                                    &&(j+1<=robot_radius+1)
                                    &&(j<=robot_radius+1))
                            {
                                path.emplace_back(Point2D(floor[i+(j)].x, floor[i+(j)].y));

                                delta = floor[i+(j+1)].y-floor[i+(j)].y;

                                increment = delta/abs(delta);
                                for(int k = 0; k <= abs(delta); k++)
                                {
                                    path.emplace_back(Point2D(floor[i+(j+1)].x, cell.floor[i+(j+1)].y-abs(delta) +increment*(k)));
                                }
                            }
                            else
                            {
                                path.emplace_back(floor[i+(j)]);
                            }

                        }
                    }

                    reverse = !reverse;
                }
                else
                {
                    y_start = floor[i].y;
                    y_end   = ceiling[i].y;

                    for (y = y_start; y >= y_end; y--)
                    {
                        path.emplace_back(Point2D(x, y));
                    }

                    if((std::abs(ceiling[i+1].y-ceiling[i].y)>=2)&&(i+1<ceiling.size()))
                    {
                        delta = ceiling[i+1].y-ceiling[i].y;
                        increment = delta/abs(delta);
                        for(int k = 1; k <= abs(delta); k++)
                        {
                            path.emplace_back(Point2D(ceiling[i].x, ceiling[i].y+increment*(k)));
                        }
                    }

                    if(robot_radius != 0)
                    {
                        for(int j = 1; j <= robot_radius+1; j++)
                        {
                            // 沿着ceiling从左往右
                            if(x+j >= ceiling.back().x)
                            {
                                i = i - (robot_radius - (j - 1));
                                break;
                            }

                            // 提前转
                            else if((ceiling[i+(j+1)].y-ceiling[i+(j)].y>=2)
                               &&(j+1 <= robot_radius+1)
                               &&(j <= robot_radius+1))
                            {
                                delta = ceiling[i+(j+1)].y-ceiling[i+(j)].y;
                                increment = delta/abs(delta);
                                for(int k = 0; k <= abs(delta); k++)
                                {
                                    path.emplace_back(Point2D(ceiling[i+j].x, ceiling[i+j].y+increment*(k)));
                                }
                            }
                            // 滞后转
                            else if((ceiling[i+(j)].y-ceiling[i+(j+1)].y>=2)
                                    &&(j<=robot_radius+1)
                                    &&(j+1<=robot_radius+1))
                            {
                                path.emplace_back(ceiling[i+(j)]);

                                delta = ceiling[i+(j+1)].y-ceiling[i+(j)].y;

                                increment = delta/abs(delta);
                                for(int k = 0; k <= abs(delta); k++)
                                {
                                    path.emplace_back(Point2D(ceiling[i+(j+1)].x, ceiling[i+(j+1)].y+abs(delta)+increment*(k)));
                                }
                            }
                            else
                            {
                                path.emplace_back(ceiling[i+j]);
                            }

                        }
                    }

                    reverse = !reverse;
                }
            }
        }

        if(corner_indicator == TOPRIGHT)
        {
            int x=0, y=0, y_start=0, y_end=0;
            bool reverse = false;

            for(int i = ceiling.size()-1; i >= 0; i=i-(robot_radius+1))
            {
                x = ceiling[i].x;

                if(!reverse)
                {
                    y_start = ceiling[i].y;
                    y_end   = floor[i].y;

                    for(y = y_start; y <= y_end; y++)
                    {
                        path.emplace_back(Point2D(x, y));
                    }

                    if((std::abs(floor[i-1].y-floor[i].y)>=2)&&(i-1>=0))
                    {
                        delta = floor[i-1].y-floor[i].y;
                        increment = delta/abs(delta);
                        for(int k = 1; k <= abs(delta); k++)
                        {
                            path.emplace_back(Point2D(floor[i].x, floor[i].y+increment*(k)));
                        }
                    }

                    if(robot_radius != 0)
                    {
                        for(int j = 1; j <= robot_radius+1; j++)
                        {
                            // 沿着floor从右往左
                            if(x-j <= floor.front().x)
                            {
                                i = i + (robot_radius - (j - 1));
                                break;
                            }
                            //提前转
                            else if((floor[i-(j)].y-floor[i-(j+1)].y>=2)
                               &&(j<=robot_radius+1)
                               &&(j+1<=robot_radius+1))
                            {
                                delta = floor[i-(j+1)].y-floor[i-(j)].y;
                                increment = delta/abs(delta);
                                for(int k = 0; k <= abs(delta); k++)
                                {
                                    path.emplace_back(Point2D(floor[i-(j)].x, floor[i-(j)].y+increment*(k)));
                                }
                            }
                            //滞后转
                            else if((floor[i-(j+1)].y-floor[i-(j)].y>=2)
                                    &&(j+1<=robot_radius+1)
                                    &&(j<=robot_radius+1))
                            {
                                path.emplace_back(Point2D(floor[i-(j)].x, floor[i-(j)].y));

                                delta = floor[i-(j+1)].y-floor[i-(j)].y;

                                increment = delta/abs(delta);
                                for(int k = 0; k <= abs(delta); k++)
                                {
                                    path.emplace_back(Point2D(floor[i-(j+1)].x, cell.floor[i-(j+1)].y-abs(delta) +increment*(k)));
                                }
                            }
                            else
                            {
                                path.emplace_back(floor[i-(j)]);
                            }
                        }
                    }

                    reverse = !reverse;
                }
                else
                {
                    y_start = floor[i].y;
                    y_end   = ceiling[i].y;

                    for (y = y_start; y >= y_end; y--)
                    {
                        path.emplace_back(Point2D(x, y));
                    }

                    if((std::abs(ceiling[i-1].y-ceiling[i].y)>=2)&&(i-1>=0))
                    {
                        delta = ceiling[i-1].y-ceiling[i].y;
                        increment = delta/abs(delta);
                        for(int k = 1; k <= abs(delta); k++)
                        {
                            path.emplace_back(Point2D(ceiling[i].x, ceiling[i].y+increment*(k)));
                        }
                    }

                    if(robot_radius != 0)
                    {
                        for(int j = 1; j <= robot_radius+1; j++)
                        {
                            // 沿着ceiling从右往左
                            if( x-j <= ceiling.front().x)
                            {
                                i = i + (robot_radius - (j - 1));
                                break;
                            }
                            // 提前转
                            else if((ceiling[i-(j+1)].y-ceiling[i-(j)].y>=2)
                               &&(j+1 <= robot_radius+1)
                               &&(j <= robot_radius+1))
                            {
                                delta = ceiling[i-(j+1)].y-ceiling[i-(j)].y;
                                increment = delta/abs(delta);
                                for(int k = 0; k <= abs(delta); k++)
                                {
                                    path.emplace_back(Point2D(ceiling[i-j].x, ceiling[i-j].y+increment*(k)));
                                }
                            }
                            // 滞后转
                            else if((ceiling[i-(j)].y-ceiling[i-(j+1)].y>=2)
                                    &&(j<=robot_radius+1)
                                    &&(j+1<=robot_radius+1))
                            {
                                path.emplace_back(ceiling[i-(j)]);

                                delta = ceiling[i-(j+1)].y-ceiling[i-(j)].y;

                                increment = delta/abs(delta);
                                for(int k = 0; k <= abs(delta); k++)
                                {
                                    path.emplace_back(Point2D(ceiling[i-(j+1)].x, ceiling[i-(j+1)].y+abs(delta)+increment*(k)));
                                }
                            }
                            else
                            {
                                path.emplace_back(ceiling[i-j]);
                            }
                        }
                    }

                    reverse = !reverse;
                }
            }
        }

        if(corner_indicator == BOTTOMLEFT)
        {
            int x=0, y=0, y_start=0, y_end=0;
            bool reverse = false;

            for(int i = 0; i < ceiling.size(); i=i+(robot_radius+1))
            {
                x = ceiling[i].x;

                if(!reverse)
                {
                    y_start = floor[i].y;
                    y_end   = ceiling[i].y;

                    for(y = y_start; y >= y_end; y--)
                    {
                        path.emplace_back(Point2D(x, y));
                    }

                    if((std::abs(ceiling[i+1].y-ceiling[i].y)>=2)&&(i+1<ceiling.size()))
                    {
                        delta = ceiling[i+1].y-ceiling[i].y;
                        increment = delta/abs(delta);
                        for(int k = 1; k <= abs(delta); k++)
                        {
                            path.emplace_back(Point2D(ceiling[i].x, ceiling[i].y+increment*(k)));
                        }
                    }

                    if(robot_radius != 0)
                    {
                        for(int j = 1; j <= robot_radius+1; j++)
                        {
                            // 沿着ceiling从左往右
                            if(x+j >= ceiling.back().x)
                            {
                                i = i - (robot_radius - (j - 1));
                                break;
                            }
                            // 提前转
                            else if((ceiling[i+(j+1)].y-ceiling[i+(j)].y>=2)
                               &&(j+1 <= robot_radius+1)
                               &&(j <= robot_radius+1))
                            {
                                delta = ceiling[i+(j+1)].y-ceiling[i+(j)].y;
                                increment = delta/abs(delta);
                                for(int k = 0; k <= abs(delta); k++)
                                {
                                    path.emplace_back(Point2D(ceiling[i+j].x, ceiling[i+j].y+increment*(k)));
                                }
                            }
                                // 滞后转
                            else if((ceiling[i+(j)].y-ceiling[i+(j+1)].y>=2)
                                    &&(j<=robot_radius+1)
                                    &&(j+1<=robot_radius+1))
                            {
                                path.emplace_back(ceiling[i+(j)]);

                                delta = ceiling[i+(j+1)].y-ceiling[i+(j)].y;

                                increment = delta/abs(delta);
                                for(int k = 0; k <= abs(delta); k++)
                                {
                                    path.emplace_back(Point2D(ceiling[i+(j+1)].x, ceiling[i+(j+1)].y+abs(delta)+increment*(k)));
                                }
                            }
                            else
                            {
                                path.emplace_back(ceiling[i+j]);
                            }
                        }
                    }

                    reverse = !reverse;
                }
                else
                {
                    y_start = ceiling[i].y;
                    y_end   = floor[i].y;

                    for (y = y_start; y <= y_end; y++)
                    {
                        path.emplace_back(Point2D(x, y));
                    }

                    if((std::abs(floor[i+1].y-floor[i].y)>=2)&&(i+1<floor.size()))
                    {
                        delta = floor[i+1].y-floor[i].y;
                        increment = delta/abs(delta);
                        for(int k = 1; k <= abs(delta); k++)
                        {
                            path.emplace_back(Point2D(floor[i].x, floor[i].y+increment*(k)));
                        }
                    }

                    if(robot_radius != 0)
                    {
                        for(int j = 1; j <= robot_radius+1; j++)
                        {
                            // 沿着floor从左往右
                            if(x+j >= floor.back().x)
                            {
                                i = i - (robot_radius - (j - 1));
                                break;
                            }

                            //提前转
                            else if((floor[i+(j)].y-floor[i+(j+1)].y>=2)
                               &&(j<=robot_radius+1)
                               &&(j+1<=robot_radius+1))
                            {
                                delta = floor[i+(j+1)].y-floor[i+(j)].y;
                                increment = delta/abs(delta);
                                for(int k = 0; k <= abs(delta); k++)
                                {
                                    path.emplace_back(Point2D(floor[i+(j)].x, floor[i+(j)].y+increment*(k)));
                                }
                            }
                                //滞后转
                            else if((floor[i+(j+1)].y-floor[i+(j)].y>=2)
                                    &&(j+1<=robot_radius+1)
                                    &&(j<=robot_radius+1))
                            {
                                path.emplace_back(Point2D(floor[i+(j)].x, floor[i+(j)].y));

                                delta = floor[i+(j+1)].y-floor[i+(j)].y;

                                increment = delta/abs(delta);
                                for(int k = 0; k <= abs(delta); k++)
                                {
                                    path.emplace_back(Point2D(floor[i+(j+1)].x, cell.floor[i+(j+1)].y-abs(delta) +increment*(k)));
                                }
                            }
                            else
                            {
                                path.emplace_back(floor[i+(j)]);
                            }
                        }
                    }

                    reverse = !reverse;
                }
            }
        }

        if(corner_indicator == BOTTOMRIGHT)
        {
            int x=0, y=0, y_start=0, y_end=0;
            bool reverse = false;

            for(int i = ceiling.size()-1; i >= 0; i=i-(robot_radius+1))
            {
                x = ceiling[i].x;

                if(!reverse)
                {
                    y_start = floor[i].y;
                    y_end   = ceiling[i].y;

                    for(y = y_start; y >= y_end; y--)
                    {
                        path.emplace_back(Point2D(x, y));
                    }

                    if((std::abs(ceiling[i-1].y-ceiling[i].y)>=2)&&(i-1>=0))
                    {
                        delta = ceiling[i-1].y-ceiling[i].y;
                        increment = delta/abs(delta);
                        for(int k = 1; k <= abs(delta); k++)
                        {
                            path.emplace_back(Point2D(ceiling[i].x, ceiling[i].y+increment*(k)));
                        }
                    }

                    if(robot_radius != 0)
                    {
                        for(int j = 1; j <= robot_radius+1; j++)
                        {
                            // 沿着ceiling从右往左
                            if(x-j <= ceiling.front().x)
                            {
                                i = i + (robot_radius - (j - 1));
                                break;
                            }
                            // 提前转
                            else if((ceiling[i-(j+1)].y-ceiling[i-(j)].y>=2)
                               &&(j+1 <= robot_radius+1)
                               &&(j <= robot_radius+1))
                            {
                                delta = ceiling[i-(j+1)].y-ceiling[i-(j)].y;
                                increment = delta/abs(delta);
                                for(int k = 0; k <= abs(delta); k++)
                                {
                                    path.emplace_back(Point2D(ceiling[i-j].x, ceiling[i-j].y+increment*(k)));
                                }
                            }
                                // 滞后转
                            else if((ceiling[i-(j)].y-ceiling[i-(j+1)].y>=2)
                                    &&(j<=robot_radius+1)
                                    &&(j+1<=robot_radius+1))
                            {
                                path.emplace_back(ceiling[i-(j)]);

                                delta = ceiling[i-(j+1)].y-ceiling[i-(j)].y;

                                increment = delta/abs(delta);
                                for(int k = 0; k <= abs(delta); k++)
                                {
                                    path.emplace_back(Point2D(ceiling[i-(j+1)].x, ceiling[i-(j+1)].y+abs(delta)+increment*(k)));
                                }
                            }
                            else
                            {
                                path.emplace_back(ceiling[i-j]);
                            }

                        }
                    }

                    reverse = !reverse;
                }
                else
                {
                    y_start = ceiling[i].y;
                    y_end   = floor[i].y;

                    for (y = y_start; y <= y_end; y++)
                    {
                        path.emplace_back(Point2D(x, y));
                    }

                    if((std::abs(floor[i-1].y-floor[i].y)>=2)&&(i-1>=0))
                    {
                        delta = floor[i-1].y-floor[i].y;
                        increment = delta/abs(delta);
                        for(int k = 1; k <= abs(delta); k++)
                        {
                            path.emplace_back(Point2D(floor[i].x, floor[i].y+increment*(k)));
                        }
                    }

                    if(robot_radius != 0)
                    {
                        for(int j = 1; j <= robot_radius+1; j++)
                        {
                            // 沿着floor从右往左
                            if(x-j <= floor.front().x)
                            {
                                i = i + (robot_radius - (j - 1));
                                break;
                            }
                            //提前转
                            else if((floor[i-(j)].y-floor[i-(j+1)].y>=2)
                               &&(j<=robot_radius+1)
                               &&(j+1<=robot_radius+1))
                            {
                                delta = floor[i-(j+1)].y-floor[i-(j)].y;
                                increment = delta/abs(delta);
                                for(int k = 0; k <= abs(delta); k++)
                                {
                                    path.emplace_back(Point2D(floor[i-(j)].x, floor[i-(j)].y+increment*(k)));
                                }
                            }
                                //滞后转
                            else if((floor[i-(j+1)].y-floor[i-(j)].y>=2)
                                    &&(j+1<=robot_radius+1)
                                    &&(j<=robot_radius+1))
                            {
                                path.emplace_back(Point2D(floor[i-(j)].x, floor[i-(j)].y));

                                delta = floor[i-(j+1)].y-floor[i-(j)].y;

                                increment = delta/abs(delta);
                                for(int k = 0; k <= abs(delta); k++)
                                {
                                    path.emplace_back(Point2D(floor[i-(j+1)].x, cell.floor[i-(j+1)].y-abs(delta) +increment*(k)));
                                }
                            }
                            else
                            {
                                path.emplace_back(floor[i-(j)]);
                            }

                        }
                    }

                    reverse = !reverse;
                }
            }
        }
    }

    return path;
}

std::vector<Event> bcd_planner::InitializeEventList(const Polygon& polygon, int polygon_index){
    std::vector<Event> event_list;
    for(const auto& point : polygon)
    {
        event_list.emplace_back(Event(polygon_index, point.x, point.y));
    }

    return event_list;
}

void bcd_planner::AllocateObstacleEventType(const cv::Mat& map, std::vector<Event>& event_list){
    int index_offset;
    std::deque<int> in_out_index_list; 

    int N = event_list.size();

    // determine in and out and middle
    for(int i = 0; i < N; i++)
    {
        if(event_list[i].x < event_list[((i-1)%N+N)%N].x && event_list[i].x < event_list[((i+1)%N+N)%N].x)
        {
            event_list[i].event_type = IN;
            in_out_index_list.emplace_back(i);
        }

        if(event_list[i].x < event_list[((i-1)%N+N)%N].x && event_list[i].x == event_list[((i+1)%N+N)%N].x && event_list[i].y < event_list[((i+1)%N+N)%N].y)
        {
            index_offset = 2;
            while(event_list[i].x == event_list[((i+index_offset)%N+N)%N].x)
            {
                index_offset++;
            }
            if(event_list[i].x < event_list[((i+index_offset)%N+N)%N].x && event_list[i].y < event_list[((i+index_offset)%N+N)%N].y)
            {
                event_list[i].event_type = IN_TOP;  //
                in_out_index_list.emplace_back(i);
            }
        }

        if(event_list[i].x == event_list[((i-1)%N+N)%N].x && event_list[i].x < event_list[((i+1)%N+N)%N].x && event_list[i].y < event_list[((i-1)%N+N)%N].y)
        {
            index_offset = 2;
            while(event_list[i].x == event_list[((i-index_offset)%N+N)%N].x)
            {
                index_offset++;
            }
            if(event_list[i].x < event_list[((i-index_offset)%N+N)%N].x && event_list[i].y < event_list[((i-index_offset)%N+N)%N].y)
            {
                event_list[i].event_type = IN_TOP; 
                in_out_index_list.emplace_back(i);
            }
        }

       
        if(event_list[i].x < event_list[((i-1)%N+N)%N].x && event_list[i].x == event_list[((i+1)%N+N)%N].x && event_list[i].y > event_list[((i+1)%N+N)%N].y)
        {
            index_offset = 2;
            while(event_list[i].x == event_list[((i+index_offset)%N+N)%N].x)
            {
                index_offset++;
            }
            if(event_list[i].x < event_list[((i+index_offset)%N+N)%N].x && event_list[i].y > event_list[((i+index_offset)%N+N)%N].y)
            {
                event_list[i].event_type = IN_BOTTOM;
                in_out_index_list.emplace_back(i);
            }
        }

        if(event_list[i].x == event_list[((i-1)%N+N)%N].x && event_list[i].x < event_list[((i+1)%N+N)%N].x && event_list[i].y > event_list[((i-1)%N+N)%N].y)
        {
            index_offset = 2;
            while(event_list[i].x == event_list[((i-index_offset)%N+N)%N].x)
            {
                index_offset++;
            }
            if(event_list[i].x < event_list[((i-index_offset)%N+N)%N].x && event_list[i].y > event_list[((i-index_offset)%N+N)%N].y)
            {
                event_list[i].event_type = IN_BOTTOM;
                in_out_index_list.emplace_back(i);
            }
        }

        if(event_list[i].x == event_list[((i-1)%N+N)%N].x && event_list[i].x == event_list[((i+1)%N+N)%N].x)
        {
            event_list[i].event_type = MIDDLE;  
        }


        if(event_list[i].x > event_list[((i-1)%N+N)%N].x && event_list[i].x > event_list[((i+1)%N+N)%N].x)
        {
            event_list[i].event_type = OUT;
            in_out_index_list.emplace_back(i);
        }


        if(event_list[i].x == event_list[((i-1)%N+N)%N].x && event_list[i].x > event_list[((i+1)%N+N)%N].x && event_list[i].y < event_list[((i-1)%N+N)%N].y)
        {
            index_offset = 2;
            while(event_list[i].x == event_list[((i-index_offset)%N+N)%N].x)
            {
                index_offset++;
            }
            if(event_list[i].x > event_list[((i-index_offset)%N+N)%N].x && event_list[i].y < event_list[((i-index_offset)%N+N)%N].y)
            {
                event_list[i].event_type = OUT_TOP;

            }
        }

        if(event_list[i].x > event_list[((i-1)%N+N)%N].x && event_list[i].x == event_list[((i+1)%N+N)%N].x && event_list[i].y < event_list[((i+1)%N+N)%N].y)
        {
            index_offset = 2;
            while(event_list[i].x == event_list[((i+index_offset)%N+N)%N].x)
            {
                index_offset++;
            }
            if(event_list[i].x > event_list[((i+index_offset)%N+N)%N].x && event_list[i].y < event_list[((i+index_offset)%N+N)%N].y)
            {
                event_list[i].event_type = OUT_TOP;
                in_out_index_list.emplace_back(i);
                
            }
        }

        if(event_list[i].x == event_list[((i-1)%N+N)%N].x && event_list[i].x > event_list[((i+1)%N+N)%N].x && event_list[i].y > event_list[((i-1)%N+N)%N].y)
        {
            index_offset = 2;
            while(event_list[i].x == event_list[((i-index_offset)%N+N)%N].x)
            {
                index_offset++;
            }
            if(event_list[i].x > event_list[((i-index_offset)%N+N)%N].x && event_list[i].y > event_list[((i-index_offset)%N+N)%N].y)
            {
                event_list[i].event_type = OUT_BOTTOM;
                in_out_index_list.emplace_back(i);
            }
        }


        if(event_list[i].x > event_list[((i-1)%N+N)%N].x && event_list[i].x == event_list[((i+1)%N+N)%N].x && event_list[i].y > event_list[((i+1)%N+N)%N].y)
        {
            index_offset = 2;
            while(event_list[i].x == event_list[((i+index_offset)%N+N)%N].x)
            {
                index_offset++;
            }
            if(event_list[i].x > event_list[((i+index_offset)%N+N)%N].x && event_list[i].y > event_list[((i+index_offset)%N+N)%N].y)
            {
                event_list[i].event_type = OUT_BOTTOM;       //ok
                in_out_index_list.emplace_back(i);
            }
        }
    }

    // determine inner
    Point2D neighbor_point;
    int temp_index;

    for(auto in_out_index : in_out_index_list)
    {
        if(event_list[in_out_index].event_type == OUT)
        {
            neighbor_point = Point2D(event_list[in_out_index].x+1, event_list[in_out_index].y);
            if(map.at<cv::Vec3b>(neighbor_point.y, neighbor_point.x) == cv::Vec3b(0,0,0))           
            {
                event_list[in_out_index].event_type = INNER_OUT;           
            }
        }

        if(event_list[in_out_index].event_type == OUT_TOP)
        {
            neighbor_point = Point2D(event_list[in_out_index].x+1, event_list[in_out_index].y);    
            if(map.at<cv::Vec3b>(neighbor_point.y, neighbor_point.x) == cv::Vec3b(0,0,0))
            {
                event_list[in_out_index].event_type = INNER_OUT_TOP;
            }
        }

        if(event_list[in_out_index].event_type == OUT_BOTTOM)
        {
            neighbor_point = Point2D(event_list[in_out_index].x+1, event_list[in_out_index].y);
            if(map.at<cv::Vec3b>(neighbor_point.y, neighbor_point.x) == cv::Vec3b(0,0,0))
            {
                event_list[in_out_index].event_type = INNER_OUT_BOTTOM;
            }

        }

        if(event_list[in_out_index].event_type == IN)
        {
            neighbor_point = Point2D(event_list[in_out_index].x-1, event_list[in_out_index].y);
            if(map.at<cv::Vec3b>(neighbor_point.y, neighbor_point.x) == cv::Vec3b(0,0,0))
            {
                event_list[in_out_index].event_type = INNER_IN;
            }
        }


        if(event_list[in_out_index].event_type == IN_TOP)
        {
            neighbor_point = Point2D(event_list[in_out_index].x-1, event_list[in_out_index].y);
            if(map.at<cv::Vec3b>(neighbor_point.y, neighbor_point.x) == cv::Vec3b(0,0,0))
            {
                event_list[in_out_index].event_type = INNER_IN_TOP;
            }
        }

        if(event_list[in_out_index].event_type == IN_BOTTOM)
        {
            neighbor_point = Point2D(event_list[in_out_index].x-1, event_list[in_out_index].y);
            if(map.at<cv::Vec3b>(neighbor_point.y, neighbor_point.x) == cv::Vec3b(0,0,0))
            {
                event_list[in_out_index].event_type = INNER_IN_BOTTOM;
            }
        }
    }

    // determine floor and ceiling
    std::deque<int> ceiling_floor_index_list;    
    for(int i = 0; i < in_out_index_list.size(); i++)
    {
        if(
                (event_list[in_out_index_list[0]].event_type==OUT
                 ||event_list[in_out_index_list[0]].event_type==OUT_TOP
                 ||event_list[in_out_index_list[0]].event_type==OUT_BOTTOM
                 ||event_list[in_out_index_list[0]].event_type==INNER_OUT
                 ||event_list[in_out_index_list[0]].event_type==INNER_OUT_TOP
                 ||event_list[in_out_index_list[0]].event_type==INNER_OUT_BOTTOM)
                &&
                (event_list[in_out_index_list[1]].event_type==IN
                 ||event_list[in_out_index_list[1]].event_type==IN_TOP
                 ||event_list[in_out_index_list[1]].event_type==IN_BOTTOM
                 ||event_list[in_out_index_list[1]].event_type==INNER_IN
                 ||event_list[in_out_index_list[1]].event_type==INNER_IN_TOP
                 ||event_list[in_out_index_list[1]].event_type==INNER_IN_BOTTOM)
                )
        {
            if(in_out_index_list[0] < in_out_index_list[1])
            {
                for(int j = in_out_index_list[0]+1; j < in_out_index_list[1]; j++)
                {
                    if(event_list[j].event_type != MIDDLE)
                    {
                        event_list[j].event_type = FLOOR;
                        ceiling_floor_index_list.emplace_back(j);
                    }
                }
            }
            else
            {
                for(int j = in_out_index_list[0]+1; j < event_list.size(); j++)
                {
                    if(event_list[j].event_type != MIDDLE)
                    {
                        event_list[j].event_type = FLOOR;
                        ceiling_floor_index_list.emplace_back(j);
                    }
                }
                for(int k = 0; k < in_out_index_list[1]; k++)
                {
                    if(event_list[k].event_type != MIDDLE)
                    {
                        event_list[k].event_type = FLOOR;
                        ceiling_floor_index_list.emplace_back(k);
                    }
                }
            }
        }

        if(
                (event_list[in_out_index_list[0]].event_type==IN
                 ||event_list[in_out_index_list[0]].event_type==IN_TOP
                 ||event_list[in_out_index_list[0]].event_type==IN_BOTTOM
                 ||event_list[in_out_index_list[0]].event_type==INNER_IN
                 ||event_list[in_out_index_list[0]].event_type==INNER_IN_TOP
                 ||event_list[in_out_index_list[0]].event_type==INNER_IN_BOTTOM)
                &&
                (event_list[in_out_index_list[1]].event_type==OUT
                 ||event_list[in_out_index_list[1]].event_type==OUT_TOP
                 ||event_list[in_out_index_list[1]].event_type==OUT_BOTTOM
                 ||event_list[in_out_index_list[1]].event_type==INNER_OUT
                 ||event_list[in_out_index_list[1]].event_type==INNER_OUT_TOP
                 ||event_list[in_out_index_list[1]].event_type==INNER_OUT_BOTTOM)
                )
        {
            if(in_out_index_list[0] < in_out_index_list[1])
            {
                for(int j = in_out_index_list[0]+1; j < in_out_index_list[1]; j++)
                {
                    if(event_list[j].event_type != MIDDLE)
                    {
                        event_list[j].event_type = CEILING;
                        ceiling_floor_index_list.emplace_back(j);
                    }
                }
            }
            else
            {
                for(int j = in_out_index_list[0]+1; j < event_list.size(); j++)
                {
                    if(event_list[j].event_type != MIDDLE)
                    {
                        event_list[j].event_type = CEILING;
                        ceiling_floor_index_list.emplace_back(j);
                    }
                }
                for(int k = 0; k < in_out_index_list[1]; k++)
                {
                    if(event_list[k].event_type != MIDDLE)
                    {
                        event_list[k].event_type = CEILING;
                        ceiling_floor_index_list.emplace_back(k);
                    }
                }
            }
        }

        temp_index = in_out_index_list.front();
        in_out_index_list.pop_front();
        in_out_index_list.emplace_back(temp_index);
    }


    // filter ceiling and floor
    for(int i = 0; i < ceiling_floor_index_list.size()-1; i++)
    {
        if(event_list[ceiling_floor_index_list[i]].event_type==CEILING
           && event_list[ceiling_floor_index_list[i+1]].event_type==CEILING
           && event_list[ceiling_floor_index_list[i]].x==event_list[ceiling_floor_index_list[i+1]].x)
        {
            if(event_list[ceiling_floor_index_list[i]].y>event_list[ceiling_floor_index_list[i+1]].y)
            {
                event_list[ceiling_floor_index_list[i+1]].event_type = MIDDLE;
            }
            else
            {
                event_list[ceiling_floor_index_list[i]].event_type = MIDDLE;
            }
        }
        if(event_list[ceiling_floor_index_list[i]].event_type==FLOOR
           && event_list[ceiling_floor_index_list[i+1]].event_type==FLOOR
           && event_list[ceiling_floor_index_list[i]].x==event_list[ceiling_floor_index_list[i+1]].x)
        {
            if(event_list[ceiling_floor_index_list[i]].y<event_list[ceiling_floor_index_list[i+1]].y)
            {
                event_list[ceiling_floor_index_list[i+1]].event_type = MIDDLE;
            }
            else
            {
                event_list[ceiling_floor_index_list[i]].event_type = MIDDLE;
            }
        }
    }
    if(event_list[ceiling_floor_index_list.back()].event_type==CEILING
       && event_list[ceiling_floor_index_list.front()].event_type==CEILING
       && event_list[ceiling_floor_index_list.back()].x==event_list[ceiling_floor_index_list.front()].x)
    {
        if(event_list[ceiling_floor_index_list.back()].y>event_list[ceiling_floor_index_list.front()].y)
        {
            event_list[ceiling_floor_index_list.front()].event_type = MIDDLE;
        }
        else
        {
            event_list[ceiling_floor_index_list.back()].event_type = MIDDLE;
        }
    }
    if(event_list[ceiling_floor_index_list.back()].event_type==FLOOR
       && event_list[ceiling_floor_index_list.front()].event_type==FLOOR
       && event_list[ceiling_floor_index_list.back()].x==event_list[ceiling_floor_index_list.front()].x)
    {
        if(event_list[ceiling_floor_index_list.back()].y<event_list[ceiling_floor_index_list.front()].y)
        {
            event_list[ceiling_floor_index_list.front()].event_type = MIDDLE;
        }
        else
        {
            event_list[ceiling_floor_index_list.back()].event_type = MIDDLE;
        }
    }

}

void bcd_planner::AllocateWallEventType(const cv::Mat& map, std::vector<Event>& event_list){
    int index_offset;
    std::deque<int> in_out_index_list; 

    int N = event_list.size();

    // determine in and out and middle
    for(int i = 0; i < N; i++)
    {
        if(event_list[i].x < event_list[((i-1)%N+N)%N].x && event_list[i].x < event_list[((i+1)%N+N)%N].x)
        {
            event_list[i].event_type = IN_EX;
            in_out_index_list.emplace_back(i);
        }
        if(event_list[i].x < event_list[((i-1)%N+N)%N].x && event_list[i].x == event_list[((i+1)%N+N)%N].x && event_list[i].y < event_list[((i+1)%N+N)%N].y)
        {
            index_offset = 2;
            while(event_list[i].x == event_list[((i+index_offset)%N+N)%N].x)
            {
                index_offset++;
            }
            if(event_list[i].x < event_list[((i+index_offset)%N+N)%N].x && event_list[i].y < event_list[((i+index_offset)%N+N)%N].y)
            {
                event_list[i].event_type = IN_TOP_EX;
                in_out_index_list.emplace_back(i);
            }
        }

        if(event_list[i].x == event_list[((i-1)%N+N)%N].x && event_list[i].x < event_list[((i+1)%N+N)%N].x && event_list[i].y < event_list[((i-1)%N+N)%N].y)
        {
            index_offset = 2;
            while(event_list[i].x == event_list[((i-index_offset)%N+N)%N].x)
            {
                index_offset++;
            }
            if(event_list[i].x < event_list[((i-index_offset)%N+N)%N].x && event_list[i].y < event_list[((i-index_offset)%N+N)%N].y)
            {
                event_list[i].event_type = IN_TOP_EX;
                in_out_index_list.emplace_back(i);
            }
        }

        if(event_list[i].x < event_list[((i-1)%N+N)%N].x && event_list[i].x == event_list[((i+1)%N+N)%N].x && event_list[i].y > event_list[((i+1)%N+N)%N].y)
        {
            index_offset = 2;
            while(event_list[i].x == event_list[((i+index_offset)%N+N)%N].x)
            {
                index_offset++;
            }
            if(event_list[i].x < event_list[((i+index_offset)%N+N)%N].x && event_list[i].y > event_list[((i+index_offset)%N+N)%N].y)
            {
                event_list[i].event_type = IN_BOTTOM_EX;
                in_out_index_list.emplace_back(i);
            }
        }

        if(event_list[i].x == event_list[((i-1)%N+N)%N].x && event_list[i].x < event_list[((i+1)%N+N)%N].x && event_list[i].y > event_list[((i-1)%N+N)%N].y)
        {
            index_offset = 2;
            while(event_list[i].x == event_list[((i-index_offset)%N+N)%N].x)
            {
                index_offset++;
            }
            if(event_list[i].x < event_list[((i-index_offset)%N+N)%N].x && event_list[i].y > event_list[((i-index_offset)%N+N)%N].y)
            {
                event_list[i].event_type = IN_BOTTOM_EX;
                in_out_index_list.emplace_back(i);
            }
        }

        if(event_list[i].x == event_list[((i-1)%N+N)%N].x && event_list[i].x == event_list[((i+1)%N+N)%N].x)
        {
            event_list[i].event_type = MIDDLE;
        }


        if(event_list[i].x > event_list[((i-1)%N+N)%N].x && event_list[i].x > event_list[((i+1)%N+N)%N].x)
        {
            event_list[i].event_type = OUT_EX;
            in_out_index_list.emplace_back(i);
        }


        if(event_list[i].x == event_list[((i-1)%N+N)%N].x && event_list[i].x > event_list[((i+1)%N+N)%N].x && event_list[i].y < event_list[((i-1)%N+N)%N].y)
        {
            index_offset = 2;
            while(event_list[i].x == event_list[((i-index_offset)%N+N)%N].x)
            {
                index_offset++;
            }
            if(event_list[i].x > event_list[((i-index_offset)%N+N)%N].x && event_list[i].y < event_list[((i-index_offset)%N+N)%N].y)
            {
                event_list[i].event_type = OUT_TOP_EX;
                in_out_index_list.emplace_back(i);
            }
        }

        if(event_list[i].x > event_list[((i-1)%N+N)%N].x && event_list[i].x == event_list[((i+1)%N+N)%N].x && event_list[i].y < event_list[((i+1)%N+N)%N].y)
        {
            index_offset = 2;
            while(event_list[i].x == event_list[((i+index_offset)%N+N)%N].x)
            {
                index_offset++;
            }
            if(event_list[i].x > event_list[((i+index_offset)%N+N)%N].x && event_list[i].y < event_list[((i+index_offset)%N+N)%N].y)
            {
                event_list[i].event_type = OUT_TOP_EX;
                in_out_index_list.emplace_back(i);
            }
        }

        if(event_list[i].x == event_list[((i-1)%N+N)%N].x && event_list[i].x > event_list[((i+1)%N+N)%N].x && event_list[i].y > event_list[((i-1)%N+N)%N].y)
        {
            index_offset = 2;
            while(event_list[i].x == event_list[((i-index_offset)%N+N)%N].x)
            {
                index_offset++;
            }
            if(event_list[i].x > event_list[((i-index_offset)%N+N)%N].x && event_list[i].y > event_list[((i-index_offset)%N+N)%N].y)
            {
                event_list[i].event_type = OUT_BOTTOM_EX;
                in_out_index_list.emplace_back(i);
            }
        }


        if(event_list[i].x > event_list[((i-1)%N+N)%N].x && event_list[i].x == event_list[((i+1)%N+N)%N].x && event_list[i].y > event_list[((i+1)%N+N)%N].y)
        {
            index_offset = 2;
            while(event_list[i].x == event_list[((i+index_offset)%N+N)%N].x)
            {
                index_offset++;
            }
            if(event_list[i].x > event_list[((i+index_offset)%N+N)%N].x && event_list[i].y > event_list[((i+index_offset)%N+N)%N].y)
            {
                event_list[i].event_type = OUT_BOTTOM_EX;
                in_out_index_list.emplace_back(i);
            }
        }
    }

    // determine inner
    Point2D neighbor_point;   
    int temp_index;
    for(auto in_out_index : in_out_index_list)
    {
        if(event_list[in_out_index].event_type == OUT_EX)
        {
            neighbor_point = Point2D(event_list[in_out_index].x+1, event_list[in_out_index].y);
            if(map.at<cv::Vec3b>(neighbor_point.y, neighbor_point.x) == cv::Vec3b(255,255,255) && neighbor_point.x < map.cols) 
            {
                event_list[in_out_index].event_type = INNER_OUT_EX;
            }
        }

        if(event_list[in_out_index].event_type == OUT_TOP_EX)
        {
            neighbor_point = Point2D(event_list[in_out_index].x+1, event_list[in_out_index].y);
            if(map.at<cv::Vec3b>(neighbor_point.y, neighbor_point.x) == cv::Vec3b(255,255,255) && neighbor_point.x < map.cols)
            {
                event_list[in_out_index].event_type = INNER_OUT_TOP_EX;
            }
        }

        if(event_list[in_out_index].event_type == OUT_BOTTOM_EX)
        {
            neighbor_point = Point2D(event_list[in_out_index].x+1, event_list[in_out_index].y);
            if(map.at<cv::Vec3b>(neighbor_point.y, neighbor_point.x) == cv::Vec3b(255,255,255) && neighbor_point.x < map.cols)
            {
                event_list[in_out_index].event_type = INNER_OUT_BOTTOM_EX;
            }

        }

        if(event_list[in_out_index].event_type == IN_EX)
        {
            neighbor_point = Point2D(event_list[in_out_index].x-1, event_list[in_out_index].y);
            if(map.at<cv::Vec3b>(neighbor_point.y, neighbor_point.x) == cv::Vec3b(255,255,255) && neighbor_point.x>=0)
            {
                event_list[in_out_index].event_type = INNER_IN_EX;
            }
        }


        if(event_list[in_out_index].event_type == IN_TOP_EX)
        {
            neighbor_point = Point2D(event_list[in_out_index].x-1, event_list[in_out_index].y);
            if(map.at<cv::Vec3b>(neighbor_point.y, neighbor_point.x) == cv::Vec3b(255,255,255) && neighbor_point.x>=0)
            {
                event_list[in_out_index].event_type = INNER_IN_TOP_EX;
            }
        }

        if(event_list[in_out_index].event_type == IN_BOTTOM_EX)
        {
            neighbor_point = Point2D(event_list[in_out_index].x-1, event_list[in_out_index].y);
            if(map.at<cv::Vec3b>(neighbor_point.y, neighbor_point.x) == cv::Vec3b(255,255,255) && neighbor_point.x>=0)
            {
                event_list[in_out_index].event_type = INNER_IN_BOTTOM_EX;
            }
        }
    }

    // determine floor and ceiling
    std::deque<int> ceiling_floor_index_list;

    for(int i = 0; i < in_out_index_list.size(); i++)
    {
        if(
                (event_list[in_out_index_list[0]].event_type==OUT_EX
                 ||event_list[in_out_index_list[0]].event_type==OUT_TOP_EX
                 ||event_list[in_out_index_list[0]].event_type==OUT_BOTTOM_EX
                 ||event_list[in_out_index_list[0]].event_type==INNER_OUT_EX
                 ||event_list[in_out_index_list[0]].event_type==INNER_OUT_TOP_EX
                 ||event_list[in_out_index_list[0]].event_type==INNER_OUT_BOTTOM_EX)
                &&
                (event_list[in_out_index_list[1]].event_type==IN_EX
                 ||event_list[in_out_index_list[1]].event_type==IN_TOP_EX
                 ||event_list[in_out_index_list[1]].event_type==IN_BOTTOM_EX
                 ||event_list[in_out_index_list[1]].event_type==INNER_IN_EX
                 ||event_list[in_out_index_list[1]].event_type==INNER_IN_TOP_EX
                 ||event_list[in_out_index_list[1]].event_type==INNER_IN_BOTTOM_EX)
                )
        {
            if(in_out_index_list[0] < in_out_index_list[1])
            {
                for(int j = in_out_index_list[0]+1; j < in_out_index_list[1]; j++)
                {
                    if(event_list[j].event_type != MIDDLE)
                    {
                        event_list[j].event_type = CEILING;
                        ceiling_floor_index_list.emplace_back(j);
                    }
                }
            }
            else
            {
                for(int j = in_out_index_list[0]+1; j < event_list.size(); j++)
                {
                    if(event_list[j].event_type != MIDDLE)
                    {
                        event_list[j].event_type = CEILING;
                        ceiling_floor_index_list.emplace_back(j);
                    }
                }
                for(int k = 0; k < in_out_index_list[1]; k++)
                {
                    if(event_list[k].event_type != MIDDLE)
                    {
                        event_list[k].event_type = CEILING;
                        ceiling_floor_index_list.emplace_back(k);
                    }
                }
            }
        }

        if(
                (event_list[in_out_index_list[0]].event_type==IN_EX
                 ||event_list[in_out_index_list[0]].event_type==IN_TOP_EX
                 ||event_list[in_out_index_list[0]].event_type==IN_BOTTOM_EX
                 ||event_list[in_out_index_list[0]].event_type==INNER_IN_EX
                 ||event_list[in_out_index_list[0]].event_type==INNER_IN_TOP_EX
                 ||event_list[in_out_index_list[0]].event_type==INNER_IN_BOTTOM_EX)
                &&
                (event_list[in_out_index_list[1]].event_type==OUT_EX
                 ||event_list[in_out_index_list[1]].event_type==OUT_TOP_EX
                 ||event_list[in_out_index_list[1]].event_type==OUT_BOTTOM_EX
                 ||event_list[in_out_index_list[1]].event_type==INNER_OUT_EX
                 ||event_list[in_out_index_list[1]].event_type==INNER_OUT_TOP_EX
                 ||event_list[in_out_index_list[1]].event_type==INNER_OUT_BOTTOM_EX)
                )
        {
            if(in_out_index_list[0] < in_out_index_list[1])
            {
                for(int j = in_out_index_list[0]+1; j < in_out_index_list[1]; j++)
                {
                    if(event_list[j].event_type != MIDDLE)
                    {
                        event_list[j].event_type = FLOOR;
                        ceiling_floor_index_list.emplace_back(j);
                    }
                }
            }
            else
            {
                for(int j = in_out_index_list[0]+1; j < event_list.size(); j++)
                {
                    if(event_list[j].event_type != MIDDLE)
                    {
                        event_list[j].event_type = FLOOR;
                        ceiling_floor_index_list.emplace_back(j);
                    }
                }
                for(int k = 0; k < in_out_index_list[1]; k++)
                {
                    if(event_list[k].event_type != MIDDLE)
                    {
                        event_list[k].event_type = FLOOR;
                        ceiling_floor_index_list.emplace_back(k);
                    }
                }
            }
        }

        temp_index = in_out_index_list.front();
        in_out_index_list.pop_front();
        in_out_index_list.emplace_back(temp_index);
    }


    // filter ceiling and floor
    for(int i = 0; i < ceiling_floor_index_list.size()-1; i++)
    {
        if(event_list[ceiling_floor_index_list[i]].event_type==CEILING
           && event_list[ceiling_floor_index_list[i+1]].event_type==CEILING
           && event_list[ceiling_floor_index_list[i]].x==event_list[ceiling_floor_index_list[i+1]].x)
        {
            if(event_list[ceiling_floor_index_list[i]].y>event_list[ceiling_floor_index_list[i+1]].y)
            {
                event_list[ceiling_floor_index_list[i+1]].event_type = MIDDLE;
            }
            else
            {
                event_list[ceiling_floor_index_list[i]].event_type = MIDDLE;
            }
        }
        if(event_list[ceiling_floor_index_list[i]].event_type==FLOOR
           && event_list[ceiling_floor_index_list[i+1]].event_type==FLOOR
           && event_list[ceiling_floor_index_list[i]].x==event_list[ceiling_floor_index_list[i+1]].x)
        {
            if(event_list[ceiling_floor_index_list[i]].y<event_list[ceiling_floor_index_list[i+1]].y)
            {
                event_list[ceiling_floor_index_list[i+1]].event_type = MIDDLE;
            }
            else
            {
                event_list[ceiling_floor_index_list[i]].event_type = MIDDLE;
            }
        }
    }
    if(event_list[ceiling_floor_index_list.back()].event_type==CEILING
       && event_list[ceiling_floor_index_list.front()].event_type==CEILING
       && event_list[ceiling_floor_index_list.back()].x==event_list[ceiling_floor_index_list.front()].x)
    {
        if(event_list[ceiling_floor_index_list.back()].y>event_list[ceiling_floor_index_list.front()].y)
        {
            event_list[ceiling_floor_index_list.front()].event_type = MIDDLE;
        }
        else
        {
            event_list[ceiling_floor_index_list.back()].event_type = MIDDLE;
        }
    }
    if(event_list[ceiling_floor_index_list.back()].event_type==FLOOR
       && event_list[ceiling_floor_index_list.front()].event_type==FLOOR
       && event_list[ceiling_floor_index_list.back()].x==event_list[ceiling_floor_index_list.front()].x)
    {
        if(event_list[ceiling_floor_index_list.back()].y<event_list[ceiling_floor_index_list.front()].y)
        {
            event_list[ceiling_floor_index_list.front()].event_type = MIDDLE;
        }
        else
        {
            event_list[ceiling_floor_index_list.back()].event_type = MIDDLE;
        }
    }
}

std::vector<Event> bcd_planner::GenerateObstacleEventList(const cv::Mat& map, const PolygonList& polygons){
    std::vector<Event> event_list;
    std::vector<Event> event_sublist;

    for(int i = 0; i < polygons.size(); i++)
    {
        event_sublist = InitializeEventList(polygons[i], i);
        AllocateObstacleEventType(map, event_sublist);
        event_list.insert(event_list.end(), event_sublist.begin(), event_sublist.end());
        event_sublist.clear();
    }

    std::sort(event_list.begin(), event_list.end());
    return event_list;
}

std::vector<Event> bcd_planner::GenerateWallEventList(const cv::Mat& map, const Polygon& external_contour){
    std::vector<Event> event_list;

    event_list = InitializeEventList(external_contour, INT_MAX);
    AllocateWallEventType(map, event_list);
    std::sort(event_list.begin(), event_list.end());

    return event_list;
}

std::deque<std::deque<Event>> bcd_planner::SliceListGenerator(const std::vector<Event>& wall_event_list, const std::vector<Event>& obstacle_event_list){
    std::vector<Event> event_list;
    event_list.insert(event_list.end(), obstacle_event_list.begin(), obstacle_event_list.end());
    event_list.insert(event_list.end(), wall_event_list.begin(), wall_event_list.end());
    std::sort(event_list.begin(), event_list.end());

    std::deque<std::deque<Event>> slice_list;
    std::deque<Event> slice;                
    int x = event_list.front().x;

    for(auto event : event_list)
    {
        if(event.x != x)                               
        {
            slice_list.emplace_back(slice);

            x = event.x;
            slice.clear();
            slice.emplace_back(event);
        }
        else
        {
            slice.emplace_back(event);     
        }
    }
    slice_list.emplace_back(slice);

    return slice_list;
}

void bcd_planner::ExecuteOpenOperation(std::vector<CellNode>& cell_graph, int curr_cell_idx, Point2D in, Point2D c, Point2D f, bool rewrite){
    CellNode top_cell, bottom_cell;                    
    top_cell.ceiling.emplace_back(c);
    top_cell.floor.emplace_back(in);

    bottom_cell.ceiling.emplace_back(in);
    bottom_cell.floor.emplace_back(f);

    if(!rewrite)
    {
        int top_cell_index = cell_graph.size();
        int bottom_cell_index = cell_graph.size() + 1;

        top_cell.cellIndex = top_cell_index;
        bottom_cell.cellIndex = bottom_cell_index;
        cell_graph.emplace_back(top_cell);
        cell_graph.emplace_back(bottom_cell);


        cell_graph[top_cell_index].neighbor_indices.emplace_back(curr_cell_idx);
        cell_graph[bottom_cell_index].neighbor_indices.emplace_front(curr_cell_idx);

        cell_graph[curr_cell_idx].neighbor_indices.emplace_front(top_cell_index);
        cell_graph[curr_cell_idx].neighbor_indices.emplace_front(bottom_cell_index);             
    }
    else
    {
        cell_graph[curr_cell_idx].ceiling.assign(top_cell.ceiling.begin(), top_cell.ceiling.end()); 
        cell_graph[curr_cell_idx].floor.assign(top_cell.floor.begin(), top_cell.floor.end());

        int bottom_cell_index = cell_graph.size();
        bottom_cell.cellIndex = bottom_cell_index;
        cell_graph.emplace_back(bottom_cell);

        cell_graph[cell_graph[curr_cell_idx].neighbor_indices.back()].neighbor_indices.emplace_back(bottom_cell_index);
        cell_graph[bottom_cell_index].neighbor_indices.emplace_back(cell_graph[curr_cell_idx].neighbor_indices.back());

    }
   
}

void bcd_planner::ExecuteCloseOperation(std::vector<CellNode>& cell_graph, int top_cell_idx, int bottom_cell_idx, Point2D c, Point2D f, bool rewrite){
    CellNode new_cell;                          //OUT event: 2 Old cell close.one New cell open
    new_cell.ceiling.emplace_back(c);
    new_cell.floor.emplace_back(f);

    if(!rewrite)
    {
        int new_cell_idx = cell_graph.size();
        new_cell.cellIndex = new_cell_idx;

        cell_graph.emplace_back(new_cell);


        cell_graph[new_cell_idx].neighbor_indices.emplace_back(top_cell_idx);
        cell_graph[new_cell_idx].neighbor_indices.emplace_back(bottom_cell_idx);

        cell_graph[top_cell_idx].neighbor_indices.emplace_front(new_cell_idx);
        cell_graph[bottom_cell_idx].neighbor_indices.emplace_back(new_cell_idx);
    }
    else
    {
        cell_graph[top_cell_idx].ceiling.assign(new_cell.ceiling.begin(), new_cell.ceiling.end());
        cell_graph[top_cell_idx].floor.assign(new_cell.floor.begin(), new_cell.floor.end());

        cell_graph[top_cell_idx].neighbor_indices.emplace_back(bottom_cell_idx);
        cell_graph[bottom_cell_idx].neighbor_indices.emplace_back(top_cell_idx);
    }
}

void bcd_planner::ExecuteCeilOperation(std::vector<CellNode>& cell_graph, int curr_cell_idx, const Point2D& ceil_point){
    cell_graph[curr_cell_idx].ceiling.emplace_back(ceil_point);
}

void bcd_planner::ExecuteFloorOperation(std::vector<CellNode>& cell_graph, int curr_cell_idx, const Point2D& floor_point){
    cell_graph[curr_cell_idx].floor.emplace_back(floor_point);
}

void bcd_planner::ExecuteOpenOperation(std::vector<CellNode>& cell_graph, int curr_cell_idx, Point2D in_top, Point2D in_bottom, Point2D c, Point2D f, bool rewrite){
    CellNode top_cell, bottom_cell;                         
    top_cell.ceiling.emplace_back(c);                      
    top_cell.floor.emplace_back(in_top);                       

    bottom_cell.ceiling.emplace_back(in_bottom);
    bottom_cell.floor.emplace_back(f);


    if(!rewrite)
    {
        int top_cell_index = cell_graph.size();
        int bottom_cell_index = cell_graph.size() + 1;

        top_cell.cellIndex = top_cell_index;
        bottom_cell.cellIndex = bottom_cell_index;
        cell_graph.emplace_back(top_cell);
        cell_graph.emplace_back(bottom_cell);                                              


        cell_graph[top_cell_index].neighbor_indices.emplace_back(curr_cell_idx);
        cell_graph[bottom_cell_index].neighbor_indices.emplace_front(curr_cell_idx);

        cell_graph[curr_cell_idx].neighbor_indices.emplace_front(top_cell_index);
        cell_graph[curr_cell_idx].neighbor_indices.emplace_front(bottom_cell_index);     
    }
    else                                                                                 
    {
        cell_graph[curr_cell_idx].ceiling.assign(top_cell.ceiling.begin(), top_cell.ceiling.end());
        cell_graph[curr_cell_idx].floor.assign(top_cell.floor.begin(), top_cell.floor.end());

        int bottom_cell_index = cell_graph.size();
        bottom_cell.cellIndex = bottom_cell_index;
        cell_graph.emplace_back(bottom_cell);

        cell_graph[cell_graph[curr_cell_idx].neighbor_indices.back()].neighbor_indices.emplace_back(bottom_cell_index);
        cell_graph[bottom_cell_index].neighbor_indices.emplace_back(cell_graph[curr_cell_idx].neighbor_indices.back());
    }
}


void bcd_planner::ExecuteInnerOpenOperation(std::vector<CellNode>& cell_graph, Point2D inner_in){
    CellNode new_cell;                              // open a new cell

    new_cell.ceiling.emplace_back(inner_in);        
    new_cell.floor.emplace_back(inner_in);

    int new_cell_index = cell_graph.size();

    new_cell.cellIndex = new_cell_index;
    cell_graph.emplace_back(new_cell);

}

void bcd_planner::ExecuteInnerOpenOperation(std::vector<CellNode>& cell_graph, Point2D inner_in_top, Point2D inner_in_bottom){
    CellNode new_cell;
    new_cell.ceiling.emplace_back(inner_in_top);
    new_cell.floor.emplace_back(inner_in_bottom);

    int new_cell_index = cell_graph.size();

    new_cell.cellIndex = new_cell_index;
    cell_graph.emplace_back(new_cell);
}

void bcd_planner::ExecuteInnerCloseOperation(std::vector<CellNode>& cell_graph, int curr_cell_idx, Point2D inner_out){
    cell_graph[curr_cell_idx].ceiling.emplace_back(inner_out);
    cell_graph[curr_cell_idx].floor.emplace_back(inner_out);
}

void bcd_planner::ExecuteInnerCloseOperation(std::vector<CellNode>& cell_graph, int curr_cell_idx, Point2D inner_out_top, Point2D inner_out_bottom){
    cell_graph[curr_cell_idx].ceiling.emplace_back(inner_out_top);
    cell_graph[curr_cell_idx].floor.emplace_back(inner_out_bottom);
}

void bcd_planner::DrawCells(cv::Mat& map, const CellNode& cell, cv::Scalar color){

    for(const auto& ceiling_point : cell.ceiling)
    {
        map.at<cv::Vec3b>(ceiling_point.y, ceiling_point.x) = cv::Vec3b(uchar(color[0]), uchar(color[1]), uchar(color[2]));
    }

    for(const auto& floor_point : cell.floor)
    {
        map.at<cv::Vec3b>(floor_point.y, floor_point.x) = cv::Vec3b(uchar(color[0]), uchar(color[1]), uchar(color[2]));
    }

    cv::line(map, cv::Point(cell.ceiling.front().x,cell.ceiling.front().y), cv::Point(cell.floor.front().x,cell.floor.front().y), color);
    cv::line(map, cv::Point(cell.ceiling.back().x,cell.ceiling.back().y), cv::Point(cell.floor.back().x,cell.floor.back().y), color);
}

int bcd_planner::CountCells(const std::deque<Event>& slice, int curr_idx){
    int cell_num = 0;
    for(int i = 0; i < curr_idx; i++)                      
    {
        if(
              (slice[i].event_type==IN)
           || (slice[i].event_type==IN_TOP)
           || (slice[i].event_type==INNER_IN)
           || (slice[i].event_type==INNER_IN_BOTTOM)
           || (slice[i].event_type==FLOOR)                     
           || (slice[i].event_type==IN_BOTTOM_EX)
           || (slice[i].event_type==INNER_IN_EX)
           || (slice[i].event_type==INNER_IN_TOP_EX)
          )
        {
            cell_num++;
        }
    }
    return cell_num;
}

std::deque<Event> bcd_planner::FilterSlice(const std::deque<Event>& slice){
    std::deque<Event> filtered_slice;

    for(auto event : slice)
    {
        if(event.event_type!=MIDDLE && event.event_type!=UNALLOCATED)
        {
            filtered_slice.emplace_back(event);
        }
    }
    return filtered_slice;
}

 //https://asset-pdf.scinapse.io/prod/1590932131/1590932131.pdf
void bcd_planner::ExecuteCellDecomposition(std::vector<CellNode>& cell_graph, std::vector<int>& cell_index_slice, std::vector<int>& original_cell_index_slice, const std::deque<std::deque<Event>>& slice_list){
    int curr_cell_idx = INT_MAX;                        
    int top_cell_idx = INT_MAX;                         
    int bottom_cell_idx = INT_MAX;                      

    Point2D c, f;                                      
    int c_index = INT_MAX, f_index = INT_MAX;
    int min_dist = INT_MAX;

    int event_y = INT_MAX;

    bool rewrite = false;

    std::vector<int> sub_cell_index_slices;
    std::deque<Event> curr_slice;

    int cell_counter = 0;

    for(const auto& raw_slice : slice_list)
    {
        curr_slice = FilterSlice(raw_slice);          
        original_cell_index_slice.assign(cell_index_slice.begin(), cell_index_slice.end());
 
  
        for(int j = 0; j < curr_slice.size(); j++)
        {
            if(curr_slice[j].event_type == INNER_IN_EX)
            {
                event_y = curr_slice[j].y;
                for(int k = 0; k < cell_index_slice.size(); k++)
                {
                    if(event_y > cell_graph[cell_index_slice[k]].ceiling.back().y && event_y < cell_graph[cell_index_slice[k]].floor.back().y)
                    {
                        rewrite = std::find(original_cell_index_slice.begin(), original_cell_index_slice.end(), cell_index_slice[k])==original_cell_index_slice.end(); // 若为true，则覆盖

                        min_dist = INT_MAX;
                        for(int m = 0; m < curr_slice.size(); m++)
                        {
                            if(abs(curr_slice[m].y-cell_graph[cell_index_slice[k]].ceiling.back().y)<min_dist)
                            {
                                min_dist = abs(curr_slice[m].y-cell_graph[cell_index_slice[k]].ceiling.back().y);
                                c_index = m;
                                c = Point2D(curr_slice[m].x, curr_slice[m].y);
                            }
                        }
                        curr_slice[c_index].isUsed = true;

                        min_dist = INT_MAX;
                        for(int n = 0; n < curr_slice.size(); n++)
                        {
                            if(abs(curr_slice[n].y - cell_graph[cell_index_slice[k]].floor.back().y)<min_dist)
                            {
                                min_dist = abs(curr_slice[n].y - cell_graph[cell_index_slice[k]].floor.back().y);
                                f_index = n;
                                f = Point2D(curr_slice[n].x, curr_slice[n].y);
                            }
                        }
                        curr_slice[f_index].isUsed = true;

                        curr_cell_idx = cell_index_slice[k];
                        ExecuteOpenOperation(cell_graph, curr_cell_idx,
                                                          Point2D(curr_slice[j].x, curr_slice[j].y),
                                                          c,
                                                          f,
                                                          rewrite);

                        if(!rewrite)
                        {
                            cell_index_slice.erase(cell_index_slice.begin()+k);
                            sub_cell_index_slices.clear();
                            sub_cell_index_slices = {int(cell_graph.size()-2), int(cell_graph.size()-1)};
                            cell_index_slice.insert(cell_index_slice.begin()+k, sub_cell_index_slices.begin(), sub_cell_index_slices.end());
                        }
                        else
                        {
                            cell_index_slice.insert(cell_index_slice.begin()+k+1, int(cell_graph.size()-1));
                        }

                        curr_slice[j].isUsed = true;

                        break;
                    }
                }
            }
            if(curr_slice[j].event_type == INNER_OUT_EX)
            {
                event_y = curr_slice[j].y;
                for(int k = 1; k < cell_index_slice.size(); k++)
                {
                    if(event_y > cell_graph[cell_index_slice[k-1]].ceiling.back().y && event_y < cell_graph[cell_index_slice[k]].floor.back().y)
                    {
                        rewrite = std::find(original_cell_index_slice.begin(), original_cell_index_slice.end(), cell_index_slice[k-1]) == original_cell_index_slice.end();

                        min_dist = INT_MAX;
                        for(int m = 0; m < curr_slice.size(); m++)
                        {
                            if(abs(curr_slice[m].y - cell_graph[cell_index_slice[k-1]].ceiling.back().y)<min_dist)
                            {
                                min_dist = abs(curr_slice[m].y - cell_graph[cell_index_slice[k-1]].ceiling.back().y);
                                c_index = m;
                                c = Point2D(curr_slice[m].x, curr_slice[m].y);
                            }
                        }
                        curr_slice[c_index].isUsed = true;

                        min_dist = INT_MAX;
                        for(int n = 0; n < curr_slice.size(); n++)
                        {
                            if(abs(curr_slice[n].y - cell_graph[cell_index_slice[k]].floor.back().y)<min_dist)
                            {
                                min_dist = abs(curr_slice[n].y - cell_graph[cell_index_slice[k]].floor.back().y);
                                f_index = n;
                                f = Point2D(curr_slice[n].x, curr_slice[n].y);
                            }
                        }
                        curr_slice[f_index].isUsed = true;

                        top_cell_idx = cell_index_slice[k-1];
                        bottom_cell_idx = cell_index_slice[k];

                        ExecuteCloseOperation(cell_graph, top_cell_idx, bottom_cell_idx,
                                                           c,
                                                           f,
                                                           rewrite);

                        if(!rewrite)
                        {
                            cell_index_slice.erase(cell_index_slice.begin() + k - 1);
                            cell_index_slice.erase(cell_index_slice.begin() + k - 1);
                            cell_index_slice.insert(cell_index_slice.begin() + k - 1, int(cell_graph.size() - 1));
                        }
                        else
                        {
                            cell_index_slice.erase(cell_index_slice.begin() + k);
                        }


                        curr_slice[j].isUsed = true;

                        break;
                    }
                }
            }

            if(curr_slice[j].event_type == INNER_IN_BOTTOM_EX)
            {
                event_y = curr_slice[j].y;
                for(int k = 0; k < cell_index_slice.size(); k++)
                {
                    if(event_y > cell_graph[cell_index_slice[k]].ceiling.back().y && event_y < cell_graph[cell_index_slice[k]].floor.back().y)
                    {
                        rewrite = std::find(original_cell_index_slice.begin(), original_cell_index_slice.end(), cell_index_slice[k])==original_cell_index_slice.end();

                        min_dist = INT_MAX;
                        for(int m = 0; m < curr_slice.size(); m++)
                        {
                            if(abs(curr_slice[m].y - cell_graph[cell_index_slice[k]].ceiling.back().y)<min_dist)
                            {
                                min_dist = abs(curr_slice[m].y - cell_graph[cell_index_slice[k]].ceiling.back().y);
                                c_index = m;
                                c = Point2D(curr_slice[m].x, curr_slice[m].y);
                            }
                        }
                        curr_slice[c_index].isUsed = true;

                        min_dist = INT_MAX;
                        for(int n = 0; n < curr_slice.size(); n++)
                        {
                            if(abs(curr_slice[n].y - cell_graph[cell_index_slice[k]].floor.back().y)<min_dist)
                            {
                                min_dist = abs(curr_slice[n].y - cell_graph[cell_index_slice[k]].floor.back().y);
                                f_index = n;
                                f = Point2D(curr_slice[n].x, curr_slice[n].y);
                            }
                        }
                        curr_slice[f_index].isUsed = true;

                        curr_cell_idx = cell_index_slice[k];
                        ExecuteOpenOperation(cell_graph, curr_cell_idx,
                                                          Point2D(curr_slice[j-1].x, curr_slice[j-1].y),  // in top
                                                          Point2D(curr_slice[j].x, curr_slice[j].y),      // in bottom
                                                          c,
                                                          f,
                                                          rewrite);


                        if(!rewrite)
                        {
                            cell_index_slice.erase(cell_index_slice.begin() + k);
                            sub_cell_index_slices.clear();
                            sub_cell_index_slices = {int(cell_graph.size() - 2), int(cell_graph.size() - 1)};
                            cell_index_slice.insert(cell_index_slice.begin() + k, sub_cell_index_slices.begin(),
                                                    sub_cell_index_slices.end());
                        }
                        else
                        {
                            cell_index_slice.insert(cell_index_slice.begin()+k+1, int(cell_graph.size()-1));
                        }

                        curr_slice[j-1].isUsed = true;
                        curr_slice[j].isUsed = true;

                        break;
                    }
                }
            }


            if(curr_slice[j].event_type == INNER_OUT_BOTTOM_EX)
            {
                event_y = curr_slice[j].y;
                for(int k = 1; k < cell_index_slice.size(); k++)
                {
                    if(event_y > cell_graph[cell_index_slice[k-1]].ceiling.back().y && event_y < cell_graph[cell_index_slice[k]].floor.back().y)
                    {
                        rewrite = std::find(original_cell_index_slice.begin(), original_cell_index_slice.end(), cell_index_slice[k-1]) == original_cell_index_slice.end();

                        min_dist = INT_MAX;
                        for(int m = 0; m < curr_slice.size(); m++)
                        {
                            if(abs(curr_slice[m].y - cell_graph[cell_index_slice[k-1]].ceiling.back().y)<min_dist)
                            {
                                min_dist = abs(curr_slice[m].y - cell_graph[cell_index_slice[k-1]].ceiling.back().y);
                                c_index = m;
                                c = Point2D(curr_slice[m].x, curr_slice[m].y);
                            }
                        }
                        curr_slice[c_index].isUsed = true;

                        min_dist = INT_MAX;
                        for(int n = 0; n < curr_slice.size(); n++)
                        {
                            if(abs(curr_slice[n].y - cell_graph[cell_index_slice[k]].floor.back().y)<min_dist)
                            {
                                min_dist = abs(curr_slice[n].y - cell_graph[cell_index_slice[k]].floor.back().y);
                                f_index = n;
                                f = Point2D(curr_slice[n].x, curr_slice[n].y);
                            }
                        }
                        curr_slice[f_index].isUsed = true;

                        top_cell_idx = cell_index_slice[k-1];
                        bottom_cell_idx = cell_index_slice[k];
                        ExecuteCloseOperation(cell_graph, top_cell_idx, bottom_cell_idx,
                                                           c,
                                                           f,
                                                           rewrite);

                        if(!rewrite)
                        {
                            cell_index_slice.erase(cell_index_slice.begin()+k-1);
                            cell_index_slice.erase(cell_index_slice.begin()+k-1);
                            cell_index_slice.insert(cell_index_slice.begin()+k-1, int(cell_graph.size()-1));
                        }
                        else
                        {
                            cell_index_slice.erase(cell_index_slice.begin() + k);
                        }

                        curr_slice[j-1].isUsed = true;
                        curr_slice[j].isUsed = true;

                        break;
                    }
                }
            }


            if(curr_slice[j].event_type == IN_EX)
            {
                event_y = curr_slice[j].y;

                if(!cell_index_slice.empty())
                {
                    for(int k = 1; k < cell_index_slice.size(); k++)
                    {
                        if(event_y >= cell_graph[cell_index_slice[k-1]].floor.back().y && event_y <= cell_graph[cell_index_slice[k]].ceiling.back().y)
                        {
                            ExecuteInnerOpenOperation(cell_graph, Point2D(curr_slice[j].x, curr_slice[j].y));  // inner_in
                            cell_index_slice.insert(cell_index_slice.begin()+k, int(cell_graph.size()-1));
                            curr_slice[j].isUsed = true;
                            break;
                        }
                    }
                    if(event_y <= cell_graph[cell_index_slice.front()].ceiling.back().y)
                    {
                        ExecuteInnerOpenOperation(cell_graph, Point2D(curr_slice[j].x, curr_slice[j].y));  // inner_in
                        cell_index_slice.insert(cell_index_slice.begin(), int(cell_graph.size()-1));
                        curr_slice[j].isUsed = true;
                    }
                    if(event_y >= cell_graph[cell_index_slice.back()].floor.back().y)
                    {
                        ExecuteInnerOpenOperation(cell_graph, Point2D(curr_slice[j].x, curr_slice[j].y));  // inner_in
                        cell_index_slice.insert(cell_index_slice.end(), int(cell_graph.size()-1));
                        curr_slice[j].isUsed = true;
                    }

                }
                else
                {
                    ExecuteInnerOpenOperation(cell_graph, Point2D(curr_slice[j].x, curr_slice[j].y));  // inner_in
                    cell_index_slice.emplace_back(int(cell_graph.size()-1));
                    curr_slice[j].isUsed = true;
                }

            }

         
            if(curr_slice[j].event_type == IN_BOTTOM_EX)
            {
                event_y = curr_slice[j].y;

                if(!cell_index_slice.empty())
                {
                    for(int k = 1; k < cell_index_slice.size(); k++)
                    {
                        if(event_y >= cell_graph[cell_index_slice[k-1]].floor.back().y && event_y <= cell_graph[cell_index_slice[k]].ceiling.back().y)
                        {

                            ExecuteInnerOpenOperation(cell_graph, Point2D(curr_slice[j-1].x, curr_slice[j-1].y), // inner_in_top,
                                                         Point2D(curr_slice[j].x, curr_slice[j].y));    // inner_in_bottom

                            cell_index_slice.insert(cell_index_slice.begin()+k, int(cell_graph.size()-1));

                            curr_slice[j-1].isUsed = true;
                            curr_slice[j].isUsed = true;

                            break;
                        }
                    }
                    if(event_y <= cell_graph[cell_index_slice.front()].ceiling.back().y)
                    {

                        ExecuteInnerOpenOperation(cell_graph, Point2D(curr_slice[j-1].x, curr_slice[j-1].y), // inner_in_top,
                                                     Point2D(curr_slice[j].x, curr_slice[j].y));    // inner_in_bottom

                        cell_index_slice.insert(cell_index_slice.begin(), int(cell_graph.size()-1));

                        curr_slice[j-1].isUsed = true;
                        curr_slice[j].isUsed = true;
                    }
                    if(event_y >= cell_graph[cell_index_slice.back()].floor.back().y)
                    {

                        ExecuteInnerOpenOperation(cell_graph, Point2D(curr_slice[j-1].x, curr_slice[j-1].y), // inner_in_top,
                                                     Point2D(curr_slice[j].x, curr_slice[j].y));    // inner_in_bottom

                        cell_index_slice.insert(cell_index_slice.end(), int(cell_graph.size()-1));

                        curr_slice[j-1].isUsed = true;
                        curr_slice[j].isUsed = true;
                    }
                }
                else
                {
                    //只需要考虑IN_EX和IN_BOTTOM_EX的情况，IN_BOTTOM的情况下j必然大于0
                    //当扫线从左往右开始扫时，OPEN A CELL
                    ExecuteInnerOpenOperation(cell_graph, Point2D(curr_slice[j-1].x, curr_slice[j-1].y), // inner_in_top,
                                                 Point2D(curr_slice[j].x, curr_slice[j].y));    // inner_in_bottom

                    cell_index_slice.emplace_back(int(cell_graph.size()-1));

                    curr_slice[j-1].isUsed = true;
                    curr_slice[j].isUsed = true;
                }

            }


            if(curr_slice[j].event_type == OUT_EX)
            {
                event_y = curr_slice[j].y;

                for(int k = 0; k < cell_index_slice.size(); k++)
                {
                    if(event_y >= cell_graph[cell_index_slice[k]].ceiling.back().y && event_y <= cell_graph[cell_index_slice[k]].floor.back().y)
                    {
                        curr_cell_idx = cell_index_slice[k];
                        ExecuteInnerCloseOperation(cell_graph, curr_cell_idx, Point2D(curr_slice[j].x, curr_slice[j].y));  // inner_out
                        cell_index_slice.erase(cell_index_slice.begin()+k);
                        curr_slice[j].isUsed = true;
                        break;
                    }
                }
            }

            if(curr_slice[j].event_type == OUT_BOTTOM_EX)
            {
                event_y = curr_slice[j].y;

                for(int k = 0; k < cell_index_slice.size(); k++)
                {
                    if(event_y >= cell_graph[cell_index_slice[k]].ceiling.back().y && event_y <= cell_graph[cell_index_slice[k]].floor.back().y)
                    {
                        curr_cell_idx = cell_index_slice[k];
                        ExecuteInnerCloseOperation(cell_graph, curr_cell_idx, Point2D(curr_slice[j-1].x, curr_slice[j-1].y), Point2D(curr_slice[j].x, curr_slice[j].y));  // inner_out_top, inner_out_bottom
                        cell_index_slice.erase(cell_index_slice.begin()+k);
                        curr_slice[j-1].isUsed = true;
                        curr_slice[j].isUsed = true;
                        break;
                    }
                }
            }

        }

        for(int j = 0; j < curr_slice.size(); j++)
        {
            if(curr_slice[j].event_type == IN)                          
            {
                event_y = curr_slice[j].y;
                for(int k = 0; k < cell_index_slice.size(); k++)        
                {
                    if(event_y > cell_graph[cell_index_slice[k]].ceiling.back().y && event_y < cell_graph[cell_index_slice[k]].floor.back().y)
                    {
                    
                        rewrite = std::find(original_cell_index_slice.begin(), original_cell_index_slice.end(), cell_index_slice[k])==original_cell_index_slice.end(); // 若为true，则覆盖

                        min_dist = INT_MAX;
                        for(int m = 0; m < curr_slice.size(); m++)    
                        {
                            if(abs(curr_slice[m].y-cell_graph[cell_index_slice[k]].ceiling.back().y)<min_dist)
                            {
                                min_dist = abs(curr_slice[m].y-cell_graph[cell_index_slice[k]].ceiling.back().y);
                                c_index = m;
                                c = Point2D(curr_slice[m].x, curr_slice[m].y);
                            }
                        }
                        curr_slice[c_index].isUsed = true;

                        min_dist = INT_MAX;
                        for(int n = 0; n < curr_slice.size(); n++)   
                        {
                            if(abs(curr_slice[n].y - cell_graph[cell_index_slice[k]].floor.back().y)<min_dist)
                            {
                                min_dist = abs(curr_slice[n].y - cell_graph[cell_index_slice[k]].floor.back().y);
                                f_index = n;
                                f = Point2D(curr_slice[n].x, curr_slice[n].y);
                            }
                        }
                        curr_slice[f_index].isUsed = true;

                        curr_cell_idx = cell_index_slice[k];
                        ExecuteOpenOperation(cell_graph, curr_cell_idx,                  //IN event .
                                             Point2D(curr_slice[j].x, curr_slice[j].y),      
                                             c,
                                             f,
                                             rewrite);

                        if(!rewrite) 
                        {                                                                                     
                            cell_index_slice.erase(cell_index_slice.begin()+k);
                            sub_cell_index_slices.clear();
                            sub_cell_index_slices = {int(cell_graph.size()-2), int(cell_graph.size()-1)};     
                            cell_index_slice.insert(cell_index_slice.begin()+k, sub_cell_index_slices.begin(), sub_cell_index_slices.end());
                        }
                        else
                        {
                            cell_index_slice.insert(cell_index_slice.begin()+k+1, int(cell_graph.size()-1));
                        }

                        curr_slice[j].isUsed = true;
                        break;
                    }
                }
            }
            if(curr_slice[j].event_type == OUT)
            {
                event_y = curr_slice[j].y;
                for(int k = 1; k < cell_index_slice.size(); k++)  
                {
                    if(event_y > cell_graph[cell_index_slice[k-1]].ceiling.back().y && event_y < cell_graph[cell_index_slice[k]].floor.back().y)
                    {
                        rewrite = std::find(original_cell_index_slice.begin(), original_cell_index_slice.end(), cell_index_slice[k-1]) == original_cell_index_slice.end();

                        min_dist = INT_MAX;
                        for(int m = 0; m < curr_slice.size(); m++)
                        {
                            if(abs(curr_slice[m].y - cell_graph[cell_index_slice[k-1]].ceiling.back().y)<min_dist)
                            {
                                min_dist = abs(curr_slice[m].y - cell_graph[cell_index_slice[k-1]].ceiling.back().y);
                                c_index = m;
                                c = Point2D(curr_slice[m].x, curr_slice[m].y);
                            }
                        }
                        curr_slice[c_index].isUsed = true;

                        min_dist = INT_MAX;
                        for(int n = 0; n < curr_slice.size(); n++)
                        {
                            if(abs(curr_slice[n].y - cell_graph[cell_index_slice[k]].floor.back().y)<min_dist)
                            {
                                min_dist = abs(curr_slice[n].y - cell_graph[cell_index_slice[k]].floor.back().y);
                                f_index = n;
                                f = Point2D(curr_slice[n].x, curr_slice[n].y);
                            }
                        }
                        curr_slice[f_index].isUsed = true;

                        top_cell_idx = cell_index_slice[k-1];                                                        //top cell
                        bottom_cell_idx = cell_index_slice[k];                                                       //bottom cell
 
                        ExecuteCloseOperation(cell_graph, top_cell_idx, bottom_cell_idx,
                                              c,
                                              f,
                                              rewrite);

                        if(!rewrite)                                                                                 
                        {
                            cell_index_slice.erase(cell_index_slice.begin() + k - 1);                                
                            cell_index_slice.erase(cell_index_slice.begin() + k - 1);                                
                            cell_index_slice.insert(cell_index_slice.begin() + k - 1, int(cell_graph.size() - 1));   
                        }
                        else
                        {
                            cell_index_slice.erase(cell_index_slice.begin() + k);
                        }


                        curr_slice[j].isUsed = true;

                        break;
                    }
                }
            }

            if(curr_slice[j].event_type == IN_BOTTOM)
            {
                event_y = curr_slice[j].y;
                for(int k = 0; k < cell_index_slice.size(); k++)
                {
             
                    if(event_y > cell_graph[cell_index_slice[k]].ceiling.back().y && event_y < cell_graph[cell_index_slice[k]].floor.back().y) 
                    {
                        
                        rewrite = std::find(original_cell_index_slice.begin(), original_cell_index_slice.end(), cell_index_slice[k])==original_cell_index_slice.end();
                        
                        min_dist = INT_MAX;
                        for(int m = 0; m < curr_slice.size(); m++)
                        {
                            if(abs(curr_slice[m].y - cell_graph[cell_index_slice[k]].ceiling.back().y)<min_dist)
                            {
                                min_dist = abs(curr_slice[m].y - cell_graph[cell_index_slice[k]].ceiling.back().y);
                                c_index = m;
                                c = Point2D(curr_slice[m].x, curr_slice[m].y);  //ceilling point
                            }
                        }
                        curr_slice[c_index].isUsed = true;

                        min_dist = INT_MAX;
                        for(int n = 0; n < curr_slice.size(); n++)
                        {
                            if(abs(curr_slice[n].y - cell_graph[cell_index_slice[k]].floor.back().y)<min_dist)
                            {
                                min_dist = abs(curr_slice[n].y - cell_graph[cell_index_slice[k]].floor.back().y);
                                f_index = n;
                                f = Point2D(curr_slice[n].x, curr_slice[n].y);           //floor point
                            }
                        }
                        curr_slice[f_index].isUsed = true;

                        curr_cell_idx = cell_index_slice[k];
                        ExecuteOpenOperation(cell_graph, curr_cell_idx,
                                             Point2D(curr_slice[j-1].x, curr_slice[j-1].y),  // in top
                                             Point2D(curr_slice[j].x, curr_slice[j].y),      // in bottom
                                             c,
                                             f,
                                             rewrite);


                        if(!rewrite)
                        {
                            cell_index_slice.erase(cell_index_slice.begin() + k);                                  //首先：Old Cell被删除
                            sub_cell_index_slices.clear();                                                         //当一个切片中包含两个IN_BOTTOM event时，
                            sub_cell_index_slices = {int(cell_graph.size() - 2), int(cell_graph.size() - 1)};      //会创建并添加新的Cell,这是original_cell_index_slice
                            cell_index_slice.insert(cell_index_slice.begin() + k, sub_cell_index_slices.begin(),   //所没有的
                                                    sub_cell_index_slices.end());
                        }
                        else
                        {
                            cell_index_slice.insert(cell_index_slice.begin()+k+1, int(cell_graph.size()-1));
                        }

                        curr_slice[j-1].isUsed = true;
                        curr_slice[j].isUsed = true;

                        break;
                    }
                }
            }


            if(curr_slice[j].event_type == OUT_BOTTOM)
            {
                event_y = curr_slice[j].y;
                for(int k = 1; k < cell_index_slice.size(); k++)
                {
                    if(event_y > cell_graph[cell_index_slice[k-1]].ceiling.back().y && event_y < cell_graph[cell_index_slice[k]].floor.back().y)
                    {
                        rewrite = std::find(original_cell_index_slice.begin(), original_cell_index_slice.end(), cell_index_slice[k-1]) == original_cell_index_slice.end();

                        min_dist = INT_MAX;
                        for(int m = 0; m < curr_slice.size(); m++)
                        {
                            if(abs(curr_slice[m].y - cell_graph[cell_index_slice[k-1]].ceiling.back().y)<min_dist)
                            {
                                min_dist = abs(curr_slice[m].y - cell_graph[cell_index_slice[k-1]].ceiling.back().y);
                                c_index = m;
                                c = Point2D(curr_slice[m].x, curr_slice[m].y);
                            }
                        }
                        curr_slice[c_index].isUsed = true;

                        min_dist = INT_MAX;
                        for(int n = 0; n < curr_slice.size(); n++)
                        {
                            if(abs(curr_slice[n].y - cell_graph[cell_index_slice[k]].floor.back().y)<min_dist)
                            {
                                min_dist = abs(curr_slice[n].y - cell_graph[cell_index_slice[k]].floor.back().y);
                                f_index = n;
                                f = Point2D(curr_slice[n].x, curr_slice[n].y);
                            }
                        }
                        curr_slice[f_index].isUsed = true;

                        top_cell_idx = cell_index_slice[k-1];
                        bottom_cell_idx = cell_index_slice[k];
                        ExecuteCloseOperation(cell_graph, top_cell_idx, bottom_cell_idx,
                                              c,
                                              f,
                                              rewrite);

                        if(!rewrite)
                        {
                            cell_index_slice.erase(cell_index_slice.begin()+k-1);
                            cell_index_slice.erase(cell_index_slice.begin()+k-1);
                            cell_index_slice.insert(cell_index_slice.begin()+k-1, int(cell_graph.size()-1));
                        }
                        else
                        {
                            cell_index_slice.erase(cell_index_slice.begin() + k);
                        }

                        curr_slice[j-1].isUsed = true;
                        curr_slice[j].isUsed = true;

                        break;
                    }
                }
            }


            if(curr_slice[j].event_type == INNER_IN)
            {
                event_y = curr_slice[j].y;
                for(int k = 1; k < cell_index_slice.size(); k++)
                {
                    if(event_y >= cell_graph[cell_index_slice[k-1]].floor.back().y && event_y <= cell_graph[cell_index_slice[k]].ceiling.back().y)
                    {
                        ExecuteInnerOpenOperation(cell_graph, Point2D(curr_slice[j].x, curr_slice[j].y));  // inner_in
                        cell_index_slice.insert(cell_index_slice.begin()+k, int(cell_graph.size()-1));
                        curr_slice[j].isUsed = true;
                        break;
                    }
                }
            }

            if(curr_slice[j].event_type == INNER_IN_BOTTOM)
            {
                event_y = curr_slice[j].y;
                for(int k = 1; k < cell_index_slice.size(); k++)
                {
                    if(event_y >= cell_graph[cell_index_slice[k-1]].floor.back().y && event_y <= cell_graph[cell_index_slice[k]].ceiling.back().y)
                    {

                        ExecuteInnerOpenOperation(cell_graph, Point2D(curr_slice[j-1].x, curr_slice[j-1].y), // inner_in_top,
                                                  Point2D(curr_slice[j].x, curr_slice[j].y));    // inner_in_bottom

                        cell_index_slice.insert(cell_index_slice.begin()+k, int(cell_graph.size()-1));

                        curr_slice[j-1].isUsed = true;
                        curr_slice[j].isUsed = true;

                        break;
                    }
                }
            }


            if(curr_slice[j].event_type == INNER_OUT)
            {
                event_y = curr_slice[j].y;
                for(int k = 0; k < cell_index_slice.size(); k++)
                {
                    if(event_y >= cell_graph[cell_index_slice[k]].ceiling.back().y && event_y <= cell_graph[cell_index_slice[k]].floor.back().y)
                    {
                        curr_cell_idx = cell_index_slice[k];
                        ExecuteInnerCloseOperation(cell_graph, curr_cell_idx, Point2D(curr_slice[j].x, curr_slice[j].y));  // inner_out
                        cell_index_slice.erase(cell_index_slice.begin()+k);
                        curr_slice[j].isUsed = true;
                        break;
                    }
                }
            }

            if(curr_slice[j].event_type == INNER_OUT_BOTTOM)
            {
                event_y = curr_slice[j].y;
                for(int k = 0; k < cell_index_slice.size(); k++)
                {
                    if(event_y >= cell_graph[cell_index_slice[k]].ceiling.back().y && event_y <= cell_graph[cell_index_slice[k]].floor.back().y)
                    {
                        curr_cell_idx = cell_index_slice[k];
                        ExecuteInnerCloseOperation(cell_graph, curr_cell_idx, Point2D(curr_slice[j-1].x, curr_slice[j-1].y), Point2D(curr_slice[j].x, curr_slice[j].y));  // inner_out_top, inner_out_bottom
                        cell_index_slice.erase(cell_index_slice.begin()+k);
                        curr_slice[j-1].isUsed = true;
                        curr_slice[j].isUsed = true;
                        break;
                    }
                }
            }

        }

        
        for(int j = 0; j < curr_slice.size(); j++)
        {
            if(curr_slice[j].event_type == CEILING)
            {
                cell_counter = CountCells(curr_slice,j);  
                curr_cell_idx = cell_index_slice[cell_counter];
                if(!curr_slice[j].isUsed)
                {
                    ExecuteCeilOperation(cell_graph, curr_cell_idx, Point2D(curr_slice[j].x, curr_slice[j].y));
                }
            }
            if(curr_slice[j].event_type == FLOOR)
            {
                cell_counter = CountCells(curr_slice,j);
                curr_cell_idx = cell_index_slice[cell_counter];
                if(!curr_slice[j].isUsed)
                {
                    ExecuteFloorOperation(cell_graph, curr_cell_idx, Point2D(curr_slice[j].x, curr_slice[j].y));
                }
            }
        }
    }
   
}

Point2D bcd_planner::FindNextEntrance(const Point2D& curr_point, const CellNode& next_cell, int& corner_indicator){
    Point2D next_entrance;
    int front_x = next_cell.ceiling.front().x;
    int back_x = next_cell.ceiling.back().x;

    std::vector<Point2D> corner_points = ComputeCellCornerPoints(next_cell);

    if(abs(curr_point.x - front_x) < abs(curr_point.x - back_x))
    {
        if(abs(curr_point.y - next_cell.ceiling.front().y)<abs(curr_point.y - next_cell.floor.front().y))
        {
            next_entrance = corner_points[TOPLEFT];
            corner_indicator = TOPLEFT;
        }
        else
        {
            next_entrance = corner_points[BOTTOMLEFT];
            corner_indicator = BOTTOMLEFT;
        }
    }
    else
    {
        if(abs(curr_point.y - next_cell.ceiling.back().y)<abs(curr_point.y - next_cell.floor.back().y))
        {
            next_entrance = corner_points[TOPRIGHT];
            corner_indicator = TOPRIGHT;
        }
        else
        {
            next_entrance = corner_points[BOTTOMRIGHT];
            corner_indicator = BOTTOMRIGHT;
        }
    }

    return next_entrance;
}

std::deque<Point2D> bcd_planner::WalkInsideCell(CellNode cell, const Point2D& start, const Point2D& end){
    std::deque<Point2D> inner_path = {start};

    int start_ceiling_index_offset = start.x - cell.ceiling.front().x;
    int first_ceiling_delta_y = cell.ceiling[start_ceiling_index_offset].y - start.y;  //y1 
    int end_ceiling_index_offset = end.x - cell.ceiling.front().x;
    int second_ceiling_delta_y = end.y - cell.ceiling[end_ceiling_index_offset].y;

    int start_floor_index_offset = start.x - cell.floor.front().x;
    int first_floor_delta_y = cell.floor[start_floor_index_offset].y - start.y;
    int end_floor_index_offset = end.x - cell.floor.front().x;
    int second_floor_delta_y = end.y - cell.floor[end_floor_index_offset].y;

    if((abs(first_ceiling_delta_y)+abs(second_ceiling_delta_y)) < (abs(first_floor_delta_y)+abs(second_floor_delta_y))) //to ceiling
    {
        int first_increment_y = 0;
        if(first_ceiling_delta_y != 0)  //向上
        {
            first_increment_y = first_ceiling_delta_y / abs(first_ceiling_delta_y);

            for(int i = 1; i <= abs(first_ceiling_delta_y); i++)
            {
                inner_path.emplace_back(Point2D(start.x, start.y+(first_increment_y*i)));
            }
        }

        int delta_x = cell.ceiling[end_ceiling_index_offset].x - cell.ceiling[start_ceiling_index_offset].x;
        int increment_x = 0;
        if(delta_x != 0)              //往右
        {
            increment_x = delta_x / abs(delta_x);
        }
        for(int i = 0; i < abs(delta_x); i++)
        {
            // 提前转
            if((cell.ceiling[start_ceiling_index_offset+increment_x*(i+1)].y-cell.ceiling[start_ceiling_index_offset+increment_x*(i)].y>=2)//why 2
               &&(i+1 <= abs(delta_x))
               &&(i <= abs(delta_x)))
            {
                int delta = cell.ceiling[start_ceiling_index_offset+increment_x*(i+1)].y-cell.ceiling[start_ceiling_index_offset+increment_x*(i)].y;
                int increment = delta/abs(delta);
                for(int j = 0; j <= abs(delta); j++)
                {
                    inner_path.emplace_back(Point2D(cell.ceiling[start_ceiling_index_offset+increment_x*i].x, cell.ceiling[start_ceiling_index_offset+increment_x*i].y+increment*(j)));
                }
            }
            // 滞后转
            else if((cell.ceiling[start_ceiling_index_offset+increment_x*(i)].y-cell.ceiling[start_ceiling_index_offset+increment_x*(i+1)].y>=2)
                     &&(i<=abs(delta_x))
                     &&(i+1<=abs(delta_x)))
            {
                inner_path.emplace_back(cell.ceiling[start_ceiling_index_offset+increment_x*(i)]);

                int delta = cell.ceiling[start_ceiling_index_offset+increment_x*(i+1)].y-cell.ceiling[start_ceiling_index_offset+increment_x*(i)].y;

                int increment = delta/abs(delta);
                for(int k = 0; k <= abs(delta); k++)
                {
                    inner_path.emplace_back(Point2D(cell.ceiling[start_ceiling_index_offset+increment_x*(i+1)].x, cell.ceiling[start_ceiling_index_offset+increment_x*(i+1)].y+abs(delta)+increment*(k)));
                }
            }
            else
            {
                inner_path.emplace_back(cell.ceiling[start_ceiling_index_offset+(increment_x*i)]);
            }
        }

        int second_increment_y = 0;    //往下
        if(second_ceiling_delta_y!=0)
        {
            second_increment_y = second_ceiling_delta_y/abs(second_ceiling_delta_y);

            for(int i = 1; i <= abs(second_ceiling_delta_y); i++)
            {
                inner_path.emplace_back(Point2D(cell.ceiling[end_ceiling_index_offset].x, cell.ceiling[end_ceiling_index_offset].y+(second_increment_y*i)));
            }
        }

    }
    else // to floor
    {
        int first_increment_y = 0;
        if(first_floor_delta_y != 0)
        {
            first_increment_y = first_floor_delta_y / abs(first_floor_delta_y);

            for(int i = 1; i <= abs(first_floor_delta_y); i++)
            {
                inner_path.emplace_back(Point2D(start.x, start.y+(first_increment_y*i)));
            }
        }

        int delta_x = cell.floor[end_floor_index_offset].x - cell.floor[start_floor_index_offset].x;
        int increment_x = 0;
        if(delta_x != 0)
        {
            increment_x = delta_x / abs(delta_x);
        }
        for(int i = 0; i < abs(delta_x); i++)
        {
            //提前转
            if((cell.floor[start_floor_index_offset+increment_x*(i)].y-cell.floor[start_floor_index_offset+increment_x*(i+1)].y>=2)
               &&(i<=abs(delta_x))
               &&(i+1<=abs(delta_x)))
            {
                int delta = cell.floor[start_floor_index_offset+increment_x*(i+1)].y-cell.floor[start_floor_index_offset+increment_x*(i)].y;
                int increment = delta/abs(delta);
                for(int j = 0; j <= abs(delta); j++)
                {
                    inner_path.emplace_back(Point2D(cell.floor[start_floor_index_offset+increment_x*(i)].x, cell.floor[start_floor_index_offset+increment_x*(i)].y+increment*(j)));
                }
            }
            //滞后转
            else if((cell.floor[start_floor_index_offset+increment_x*(i+1)].y-cell.floor[start_floor_index_offset+increment_x*(i)].y>=2)
                    &&(i+1<=abs(delta_x))
                    &&(i<=abs(delta_x)))
            {
                inner_path.emplace_back(Point2D(cell.floor[start_floor_index_offset+increment_x*(i)].x, cell.floor[start_floor_index_offset+increment_x*(i)].y));

                int delta = cell.floor[start_floor_index_offset+increment_x*(i+1)].y-cell.floor[start_floor_index_offset+increment_x*(i)].y;

                int increment = delta/abs(delta);
                for(int k = 0; k <= abs(delta); k++)
                {
                    inner_path.emplace_back(Point2D(cell.floor[start_floor_index_offset+increment_x*(i+1)].x, cell.floor[start_floor_index_offset+increment_x*(i+1)].y-abs(delta) +increment*(k)));
                }
            }
            else
            {
                inner_path.emplace_back(cell.floor[start_floor_index_offset+(increment_x*i)]);
            }

        }

        int second_increment_y = 0;
        if(second_floor_delta_y!=0)
        {
            second_increment_y = second_floor_delta_y/abs(second_floor_delta_y);

            for(int i = 1; i <= abs(second_floor_delta_y); i++)
            {
                inner_path.emplace_back(Point2D(cell.floor[end_floor_index_offset].x, cell.floor[end_floor_index_offset].y+(second_increment_y*i)));
            }
        }
    }
    return inner_path;

}

std::deque<std::deque<Point2D>> bcd_planner::FindLinkingPath(const Point2D& curr_exit, Point2D& next_entrance, int& corner_indicator, CellNode curr_cell, const CellNode& next_cell){
    std::deque<std::deque<Point2D>> path;
    std::deque<Point2D> path_in_curr_cell;
    std::deque<Point2D> path_in_next_cell;

    int exit_corner_indicator = INT_MAX;
    Point2D exit = FindNextEntrance(next_entrance, curr_cell, exit_corner_indicator);
    path_in_curr_cell = WalkInsideCell(curr_cell, curr_exit, exit);

    next_entrance = FindNextEntrance(exit, next_cell, corner_indicator);

    int delta_x = next_entrance.x - exit.x;
    int delta_y = next_entrance.y - exit.y;

    int increment_x = 0;
    int increment_y = 0;

    if (delta_x != 0) {
        increment_x = delta_x / std::abs(delta_x);
    }
    if (delta_y != 0) {
        increment_y = delta_y / std::abs(delta_y);
    }

    int upper_bound = INT_MIN;
    int lower_bound = INT_MAX;

    if (exit.x >= curr_cell.ceiling.back().x)
    {
        upper_bound = curr_cell.ceiling.back().y;
        lower_bound = curr_cell.floor.back().y;
    }
    if (exit.x <= curr_cell.ceiling.front().x)
    {
        upper_bound = curr_cell.ceiling.front().y;
        lower_bound = curr_cell.floor.front().y;
    }

    if ((next_entrance.y >= upper_bound) && (next_entrance.y <= lower_bound)) 
    {
        for (int y = exit.y; y != next_entrance.y; y += increment_y) {
            path_in_curr_cell.emplace_back(Point2D(exit.x, y));
        }
        for (int x = exit.x; x != next_entrance.x; x += increment_x) {
            path_in_curr_cell.emplace_back(Point2D(x, next_entrance.y));
        }
    }
    else
    {
        for (int x = exit.x; x != next_entrance.x; x += increment_x) {
            path_in_curr_cell.emplace_back(Point2D(x, exit.y));
        }
        for (int y = exit.y; y != next_entrance.y; y += increment_y) {
            path_in_next_cell.emplace_back(Point2D(next_entrance.x, y));
        }
    }

    path = {path_in_curr_cell, path_in_next_cell};

    return path;

}

std::deque<Point2D> bcd_planner::WalkCrossCells(std::vector<CellNode>& cell_graph, std::deque<int> cell_path, const Point2D& start, const Point2D& end, int robot_radius){
    std::deque<Point2D> overall_path;
    std::deque<Point2D> sub_path;

    std::deque<std::deque<Point2D>> link_path;

    std::vector<CellNode> cells;
    cells.assign(cell_graph.begin(), cell_graph.end());

    for(auto cell : cells)
    {
        cell.isCleaned = true;
    }

    Point2D curr_exit, next_entrance;
    int curr_corner_indicator, next_corner_indicator;

    next_entrance = FindNextEntrance(start, cells[cell_path[1]], next_corner_indicator);
    curr_exit = FindNextEntrance(next_entrance, cells[cell_path[0]], curr_corner_indicator);
    sub_path = WalkInsideCell(cells[cell_path[0]], start, curr_exit);
    overall_path.insert(overall_path.end(), sub_path.begin(), sub_path.end());
    sub_path.clear();

    link_path = FindLinkingPath(curr_exit, next_entrance, next_corner_indicator, cells[cell_path[0]], cells[cell_path[1]]);
    sub_path.insert(sub_path.end(), link_path.front().begin(), link_path.front().end());
    sub_path.insert(sub_path.end(), link_path.back().begin(), link_path.back().end());


    overall_path.insert(overall_path.end(), sub_path.begin(), sub_path.end());
    sub_path.clear();

    curr_corner_indicator = next_corner_indicator;


    for(int i = 1; i < cell_path.size()-1; i++)
    {
        sub_path = GetBoustrophedonPath(cell_graph, cells[cell_path[i]], curr_corner_indicator, robot_radius);
        overall_path.insert(overall_path.end(), sub_path.begin(), sub_path.end());
        sub_path.clear();

        curr_exit = overall_path.back();
        next_entrance = FindNextEntrance(curr_exit, cells[cell_path[i+1]], next_corner_indicator);

        link_path = FindLinkingPath(curr_exit, next_entrance, next_corner_indicator, cells[cell_path[i]], cells[cell_path[i+1]]);
        sub_path.insert(sub_path.end(), link_path.front().begin(), link_path.front().end());
        sub_path.insert(sub_path.end(), link_path.back().begin(), link_path.back().end());


        overall_path.insert(overall_path.end(), sub_path.begin(), sub_path.end());
        sub_path.clear();

        curr_corner_indicator = next_corner_indicator;
    }

    sub_path = WalkInsideCell(cells[cell_path.back()], next_entrance, end);
    overall_path.insert(overall_path.end(), sub_path.begin(), sub_path.end());
    sub_path.clear();

    return overall_path;
}

std::deque<int> bcd_planner::FindShortestPath(std::vector<CellNode>& cell_graph, const Point2D& start, const Point2D& end){
    int start_cell_index = DetermineCellIndex(cell_graph, start).front();
    int end_cell_index = DetermineCellIndex(cell_graph, end).front();

    std::deque<int> cell_path = {end_cell_index};

    if(start_cell_index == end_cell_index)
    {
        return cell_path;
    }

    if(start_cell_index == end_cell_index)
    {
        cell_path.emplace_back(start_cell_index);
        return cell_path;
    }

    std::vector<CellNode> cells;
    cells.assign(cell_graph.begin(), cell_graph.end());

    for(auto cell : cells)
    {
        cell.isVisited = false;
        cell.isCleaned = false;
        cell.parentIndex = INT_MAX;
    }

    std::deque<int> search_queue = {start_cell_index};

    CellNode curr_cell;

    while(!search_queue.empty())
    {
        curr_cell = cells[search_queue.front()];

        cells[search_queue.front()].isVisited = true;
        search_queue.pop_front();

        for(int i = 0; i < curr_cell.neighbor_indices.size(); i++)
        {
            if(curr_cell.neighbor_indices[i] == end_cell_index)
            {
                cells[curr_cell.neighbor_indices[i]].parentIndex = curr_cell.cellIndex;
                search_queue.clear();
                break;
            }
            else if(!cells[curr_cell.neighbor_indices[i]].isVisited)
            {
                cells[curr_cell.neighbor_indices[i]].isVisited = true;
                cells[curr_cell.neighbor_indices[i]].parentIndex = curr_cell.cellIndex;
                search_queue.emplace_back(curr_cell.neighbor_indices[i]);
            }
        }

    }

    curr_cell = cells[end_cell_index];
    int prev_cell_index;

    while(curr_cell.parentIndex != INT_MAX)
    {
        prev_cell_index = curr_cell.parentIndex;
        cell_path.emplace_front(prev_cell_index);
        curr_cell = cells[prev_cell_index];
    }

    return cell_path;

}

void bcd_planner::InitializeColorMap(std::deque<cv::Scalar>& JetColorMap, int repeat_times){
    for(int i = 0; i <= 255; i++)
    {
        for(int j = 0; j < repeat_times; j++)
        {
            JetColorMap.emplace_back(cv::Scalar(0, i, 255));
        }
    }

    for(int i = 254; i >= 0; i--)
    {
        for(int j = 0; j < repeat_times; j++)
        {
            JetColorMap.emplace_back(cv::Scalar(0, 255, i));
        }
    }

    for(int i = 1; i <= 255; i++)
    {
        for(int j = 0; j < repeat_times; j++)
        {
            JetColorMap.emplace_back(cv::Scalar(i, 255, 0));
        }
    }

    for(int i = 254; i >= 0; i--)
    {
        for(int j = 0; j < repeat_times; j++)
        {
            JetColorMap.emplace_back(cv::Scalar(255, i, 0));
        }
    }

    for(int i = 1; i <= 255; i++)
    {
        for(int j = 0; j < repeat_times; j++)
        {
            JetColorMap.emplace_back(cv::Scalar(255, 0, i));
        }
    }

    for(int i = 254; i >= 1; i--)
    {
        for(int j = 0; j < repeat_times; j++)
        {
            JetColorMap.emplace_back(cv::Scalar(i, 0, 255));
        }
    }

}

void bcd_planner::UpdateColorMap(std::deque<cv::Scalar>& JetColorMap){
    cv::Scalar color = JetColorMap.front();
    JetColorMap.pop_front();
    JetColorMap.emplace_back(color);
}

int bcd_planner::ComputeRobotRadius(const double& meters_per_pix, const double& robot_size_in_meters){
    int robot_radius = int(robot_size_in_meters / meters_per_pix);
    return robot_radius;
}

cv::Mat1b bcd_planner::ReadMap(const std::string& map_file_path){
    cv::Mat1b original_map = cv::imread(map_file_path, CV_8U);
    return original_map;
}

cv::Mat1b bcd_planner::PreprocessMap(const cv::Mat1b& original_map){
    cv::Mat1b map = original_map.clone();
    cv::threshold(map, map, 128, 255, cv::THRESH_BINARY);
    return map;
}

void bcd_planner::ExtractRawContours(const cv::Mat& original_map, std::vector<std::vector<cv::Point>>& raw_wall_contours, std::vector<std::vector<cv::Point>>& raw_obstacle_contours){
    cv::Mat map = original_map.clone();
    cv::threshold(map, map, 128, 255, cv::THRESH_BINARY_INV);
    // cv::namedWindow("map",cv::WINDOW_NORMAL);
    // cv::imshow("map",map);
    // cv::waitKey();

    cv::cvtColor(map, map, cv::COLOR_GRAY2BGR);
    // cv::imshow("map",map);
    // cv::waitKey();
   

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(original_map.clone(), contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);
   // std::cout<<"contours size "<<contours.size()<<std::endl;

    std::vector<std::vector<cv::Point>> contours_new;
    int width = original_map.cols;
    int height = original_map.rows;
    for(int i = 0; i < contours.size(); i++){
        if(cv::contourArea(contours[i]) < (width * height - width - height - 1)){
            contours_new.push_back(contours[i]);
        }
    }
    //std::cerr << "contour_new size: " << contours_new.size() << std::endl;
    cv::Mat map_show = original_map.clone();
    cv::cvtColor(map_show, map_show, CV_GRAY2RGB);
    //cv::drawContours(map_show, contours_new, -1, cv::Scalar(0, 255, 0), 2);
    // cv::imshow("img_show", map_show);
    // cv::waitKey();

    std::vector<int> wall_cnt_indices(contours_new.size());
    std::iota(wall_cnt_indices.begin(), wall_cnt_indices.end(), 0);

//    std::sort(wall_cnt_indices.begin(), wall_cnt_indices.end(), [&contours](int lhs, int rhs){return contours[lhs].size() > contours[rhs].size();});
    std::sort(wall_cnt_indices.begin(), wall_cnt_indices.end(), [&contours_new](int lhs, int rhs){return cv::contourArea(contours_new[lhs]) > cv::contourArea(contours_new[rhs]);});

    std::vector<cv::Point> raw_wall_contour = contours_new[wall_cnt_indices.front()];
    raw_wall_contours = {raw_wall_contour};


    cv::Mat mask = cv::Mat(original_map.size(), original_map.type(), 255);
    cv::fillPoly(mask, raw_wall_contours, 0);
    // cv::imshow("map",mask);
    // cv::waitKey();

    cv::Mat base = original_map.clone();
    base += mask;
    cv::threshold(base, base, 128, 255, cv::THRESH_BINARY_INV);
    // cv::imshow("map",base);
    // cv::waitKey();  


    cv::findContours(base, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    raw_obstacle_contours = contours;
}

void bcd_planner::ExtractContours(const cv::Mat& original_map, std::vector<std::vector<cv::Point>>& wall_contours, std::vector<std::vector<cv::Point>>& obstacle_contours, int robot_radius){

    ExtractRawContours(original_map, wall_contours, obstacle_contours);

    if(robot_radius != 0)
    {
        cv::Mat3b canvas = cv::Mat3b(original_map.size(), CV_8U);
        canvas.setTo(cv::Scalar(255, 255, 255));

        cv::fillPoly(canvas, wall_contours, cv::Scalar(0, 0, 0));
        for(const auto& point:wall_contours.front())
        {
            cv::circle(canvas, point, robot_radius, cv::Scalar(255, 255, 255), -1);
        }

        cv::fillPoly(canvas, obstacle_contours, cv::Scalar(255, 255, 255));
        for(const auto& obstacle_contour:obstacle_contours)
        {
            for(const auto& point:obstacle_contour)
            {
                cv::circle(canvas, point, robot_radius, cv::Scalar(255, 255, 255), -1);
            }
        }

        cv::Mat canvas_;
        cv::cvtColor(canvas, canvas_, cv::COLOR_BGR2GRAY);
        cv::threshold(canvas_, canvas_, 200, 255, cv::THRESH_BINARY_INV);

        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(robot_radius,robot_radius), cv::Point(-1,-1));
        cv::morphologyEx(canvas_, canvas_, cv::MORPH_OPEN, kernel);

        ExtractRawContours(canvas_, wall_contours, obstacle_contours);



        std::vector<cv::Point> processed_wall_contour;
        cv::approxPolyDP(cv::Mat(wall_contours.front()), processed_wall_contour, 1, true);

        std::vector<std::vector<cv::Point>> processed_obstacle_contours(obstacle_contours.size());
        for(int i = 0; i < obstacle_contours.size(); i++)
        {
            cv::approxPolyDP(cv::Mat(obstacle_contours[i]), processed_obstacle_contours[i], 1, true);
        }

        wall_contours = {processed_wall_contour};
        obstacle_contours = processed_obstacle_contours;
    }
}

PolygonList bcd_planner::ConstructObstacles(const cv::Mat& original_map, const std::vector<std::vector<cv::Point>>& obstacle_contours){
    PolygonList obstacles;
    Polygon obstacle;

    for(const auto& obstacle_contour : obstacle_contours)
    {
        for(int j = 0; j < obstacle_contour.size()-1; j++)
        {
            cv::LineIterator line(original_map, obstacle_contour[j], obstacle_contour[j+1]);
            for(int k = 0; k < line.count-1; k++)
            {
                obstacle.emplace_back(Point2D(line.pos().x, line.pos().y));
                line++;
            }
        }
        cv::LineIterator line(original_map, obstacle_contour[obstacle_contour.size()-1], obstacle_contour[0]);
        for(int j = 0; j < line.count-1; j++)
        {
            obstacle.emplace_back(Point2D(line.pos().x, line.pos().y));
            line++;
        }

        obstacles.emplace_back(obstacle);
        obstacle.clear();
    }

    return obstacles;
}

Polygon bcd_planner::ConstructDefaultWall(const cv::Mat& original_map){
    std::vector<cv::Point> default_wall_contour = {cv::Point(0, 0), cv::Point(0, original_map.rows-1), cv::Point(original_map.cols-1, original_map.rows-1), cv::Point(original_map.cols-1, 0)};
    std::vector<std::vector<cv::Point>>default_wall_contours = {default_wall_contour};

    Polygon default_wall = ConstructObstacles(original_map, default_wall_contours).front();
    return default_wall;
}

Polygon bcd_planner::ConstructWall(const cv::Mat& original_map, std::vector<cv::Point>& wall_contour){
    Polygon wall;
    if(!wall_contour.empty())
    {
        for(int i = 0; i < wall_contour.size()-1; i++)
        {
            cv::LineIterator line(original_map, wall_contour[i], wall_contour[i+1]);
            for(int j = 0; j < line.count-1; j++)
            {
                wall.emplace_back(Point2D(line.pos().x, line.pos().y));
                line++;
            }
        }
        cv::LineIterator line(original_map, wall_contour.back(), wall_contour.front());
        for(int i = 0; i < line.count-1; i++)
        {
            wall.emplace_back(Point2D(line.pos().x, line.pos().y));
            line++;
        }

        return wall;
    }
    else
    {
        wall = ConstructDefaultWall(original_map);

        for(const auto& point : wall)
        {
            wall_contour.emplace_back(cv::Point(point.x, point.y));
        }

        return wall;
    }
}

std::vector<CellNode> bcd_planner::ConstructCellGraph(const cv::Mat& original_map, const std::vector<std::vector<cv::Point>>& wall_contours, const std::vector<std::vector<cv::Point>>& obstacle_contours, const Polygon& wall, const PolygonList& obstacles){
    cv::Mat3b map = cv::Mat3b(original_map.size());
    map.setTo(cv::Scalar(0, 0, 0));

    cv::fillPoly(map, wall_contours, cv::Scalar(255, 255, 255));
    cv::fillPoly(map, obstacle_contours, cv::Scalar(0, 0, 0));

    std::vector<Event> wall_event_list = GenerateWallEventList(map, wall);
    std::vector<Event> obstacle_event_list = GenerateObstacleEventList(map, obstacles);
    std::deque<std::deque<Event>> slice_list = SliceListGenerator(wall_event_list, obstacle_event_list);

    std::vector<CellNode> cell_graph;
    std::vector<int> cell_index_slice;
    std::vector<int> original_cell_index_slice;
    ExecuteCellDecomposition(cell_graph, cell_index_slice, original_cell_index_slice, slice_list);

    return cell_graph;
}

std::deque<std::deque<Point2D>> bcd_planner::StaticPathPlanning(const cv::Mat& map, std::vector<CellNode>& cell_graph, const Point2D& start_point, int robot_radius, bool visualize_cells, bool visualize_path, int color_repeats){
    cv::Mat3b vis_map;
    cv::cvtColor(map, vis_map, cv::COLOR_GRAY2BGR);

    std::deque<std::deque<Point2D>> global_path;
    std::deque<Point2D> local_path;
    int corner_indicator = TOPLEFT;

    int start_cell_index = DetermineCellIndex(cell_graph, start_point).front();

    std::deque<Point2D> init_path = WalkInsideCell(cell_graph[start_cell_index], start_point, ComputeCellCornerPoints(cell_graph[start_cell_index])[TOPLEFT]);
    local_path.assign(init_path.begin(), init_path.end());

    std::deque<CellNode> cell_path = GetVisittingPath(cell_graph, start_cell_index);

    if(visualize_cells||visualize_path)
    {
        cv::namedWindow("map", cv::WINDOW_NORMAL);
        cv::imshow("map", vis_map);
    }

    if(visualize_cells)
    {
        std::cout<<"cell graph has "<<cell_graph.size()<<" cells."<<std::endl;
        for(int i = 0; i < cell_graph.size(); i++)
        {
            for(int j = 0; j < cell_graph[i].neighbor_indices.size(); j++)
            {
                std::cout<<"cell "<< i << "'s neighbor: cell "<<cell_graph[cell_graph[i].neighbor_indices[j]].cellIndex<<std::endl;
            }
        }

        for(const auto& cell : cell_graph)
        {
            DrawCells(vis_map, cell);
            cv::imshow("map", vis_map);
            cv::waitKey(500);
        }
    }

    std::deque<cv::Scalar> JetColorMap;
    InitializeColorMap(JetColorMap, color_repeats);

    if(visualize_path)
    {
        cv::circle(vis_map, cv::Point(start_point.x, start_point.y), 1, cv::Scalar(0, 0, 255), -1);
        for(const auto& point : init_path)
        {
            vis_map.at<cv::Vec3b>(point.y, point.x)=cv::Vec3b(uchar(JetColorMap.front()[0]),uchar(JetColorMap.front()[1]),uchar(JetColorMap.front()[2]));
            UpdateColorMap(JetColorMap);
            cv::imshow("map", vis_map);
            cv::waitKey(1);
        }
    }

    std::deque<Point2D> inner_path;
    std::deque<std::deque<Point2D>> link_path;
    Point2D curr_exit;
    Point2D next_entrance;

    std::deque<int> return_cell_path;
    std::deque<Point2D> return_path;

    for(int i = 0; i < cell_path.size(); i++)
    {
        inner_path = GetBoustrophedonPath(cell_graph, cell_path[i], corner_indicator, robot_radius);
        local_path.insert(local_path.end(), inner_path.begin(), inner_path.end());
        if(visualize_path)
        {
            for(const auto& point : inner_path)
            {
                vis_map.at<cv::Vec3b>(point.y, point.x)=cv::Vec3b(uchar(JetColorMap.front()[0]),uchar(JetColorMap.front()[1]),uchar(JetColorMap.front()[2]));
                UpdateColorMap(JetColorMap);
                cv::imshow("map", vis_map);
                cv::waitKey(1);
            }
        }

        cell_graph[cell_path[i].cellIndex].isCleaned = true;

        if(i < (cell_path.size()-1))
        {
            curr_exit = inner_path.back();
            next_entrance = FindNextEntrance(curr_exit, cell_path[i+1], corner_indicator);
            link_path = FindLinkingPath(curr_exit, next_entrance, corner_indicator, cell_path[i], cell_path[i+1]);

            // for debugging
//            std::cout<<std::endl;
//            for(int i = 0; i < link_path.front().size(); i++)
//            {
//                int idx = DetermineCellIndex(cell_graph, link_path.front()[i]).front();
//                std::cout<<"point lies in curr cell "<<idx<<std::endl;
//            }
//
//            for(int i = 0; i < link_path.back().size(); i++)
//            {
//                int idx = DetermineCellIndex(cell_graph, link_path.back()[i]).front();
//                std::cout<<"point lies in next cell "<<idx<<std::endl;
//            }
//            std::cout<<std::endl;


            local_path.insert(local_path.end(), link_path.front().begin(), link_path.front().end());
            global_path.emplace_back(local_path);
            local_path.clear();
            local_path.insert(local_path.end(), link_path.back().begin(), link_path.back().end());


            if(visualize_path)
            {
                for(const auto& point : link_path.front())
                {
//                    vis_map.at<cv::Vec3b>(point.y, point.x)=cv::Vec3b(255, 255, 255);
                    vis_map.at<cv::Vec3b>(point.y, point.x)=cv::Vec3b(uchar(JetColorMap.front()[0]),uchar(JetColorMap.front()[1]),uchar(JetColorMap.front()[2]));
                    UpdateColorMap(JetColorMap);
                    cv::imshow("map", vis_map);
                    cv::waitKey(1);
                }

                for(const auto& point: link_path.back())
                {
//                    vis_map.at<cv::Vec3b>(point.y, point.x)=cv::Vec3b(255, 255, 255);
                    vis_map.at<cv::Vec3b>(point.y, point.x)=cv::Vec3b(uchar(JetColorMap.front()[0]),uchar(JetColorMap.front()[1]),uchar(JetColorMap.front()[2]));
                    UpdateColorMap(JetColorMap);
                    cv::imshow("map", vis_map);
                    cv::waitKey(1);
                }

            }
        }
    }
    global_path.emplace_back(local_path);

    if(visualize_cells||visualize_path)
    {
        cv::waitKey(0);
    }

    return global_path;
}

std::deque<Point2D> bcd_planner::ReturningPathPlanning(cv::Mat& map, std::vector<CellNode>& cell_graph, const Point2D& curr_pos, const Point2D& original_pos, int robot_radius, bool visualize_path){
    std::deque<int> return_cell_path = FindShortestPath(cell_graph, curr_pos, original_pos);
    std::deque<Point2D> returning_path;

    if(return_cell_path.size() == 1)
    {
        returning_path = WalkInsideCell(cell_graph[return_cell_path.front()], curr_pos, original_pos);
    }
    else
    {
        returning_path = WalkCrossCells(cell_graph, return_cell_path, curr_pos, original_pos, robot_radius);
    }

    if(visualize_path)
    {
        cv::namedWindow("map", cv::WINDOW_NORMAL);
        for(const auto& point : returning_path)
        {
            map.at<cv::Vec3b>(point.y, point.x)=cv::Vec3b(250, 250, 250);
            cv::imshow("map", map);
            cv::waitKey(1);
        }
        cv::waitKey(0);
    }

    return returning_path;
}

std::deque<Point2D> bcd_planner::FilterTrajectory(const std::deque<std::deque<Point2D>>& raw_trajectory){
    std::deque<Point2D> trajectory;

    for(const auto& sub_trajectory : raw_trajectory)
    {
        for(const auto& position : sub_trajectory)
        {
            if(!trajectory.empty())
            {
                if(position != trajectory.back())
                {
                    trajectory.emplace_back(position);
                }
            }
            else
            {
                trajectory.emplace_back(position);
            }
        }
    }

    return trajectory;

}

void bcd_planner::VisualizeTrajectory(const cv::Mat& original_map, const std::deque<Point2D>& path, int robot_radius, int vis_mode, int time_interval, int colors){
    cv::Mat3b vis_map;
    cv::cvtColor(original_map, vis_map, cv::COLOR_GRAY2BGR);

    cv::namedWindow("map", cv::WINDOW_NORMAL);

    std::deque<cv::Scalar> JetColorMap;
    int color_repeated_times = path.size()/colors + 1;
    InitializeColorMap(JetColorMap, color_repeated_times);

    switch (vis_mode)
    {
        case PATH_MODE:
            vis_map.at<cv::Vec3b>(path.front().y, path.front().x)=cv::Vec3b(uchar(JetColorMap.front()[0]),uchar(JetColorMap.front()[1]),uchar(JetColorMap.front()[2]));
            cv::imshow("map", vis_map);
            cv::waitKey(0);

            for(const auto& position:path)
            {
                vis_map.at<cv::Vec3b>(position.y, position.x)=cv::Vec3b(uchar(JetColorMap.front()[0]),uchar(JetColorMap.front()[1]),uchar(JetColorMap.front()[2]));
                UpdateColorMap(JetColorMap);
                cv::imshow("map", vis_map);
                cv::waitKey(time_interval);
            }
            break;
        case ROBOT_MODE:
            cv::circle(vis_map, cv::Point(path.front().x, path.front().y), robot_radius, cv::Scalar(255, 204, 153), -1);
            cv::imshow("map", vis_map);
            cv::waitKey(0);

            for(const auto& position:path)
            {
                cv::circle(vis_map, cv::Point(position.x, position.y), robot_radius, cv::Scalar(255, 204, 153), -1);
                cv::imshow("map", vis_map);
                cv::waitKey(time_interval);

                cv::circle(vis_map, cv::Point(position.x, position.y), robot_radius, cv::Scalar(255, 229, 204), -1);
            }
            break;
        default:
            break;
    }

    cv::waitKey(0);

}

double bcd_planner::ComputeYaw(Eigen::Vector2d curr_direction, Eigen::Vector2d base_direction){
    double yaw = std::atan2(curr_direction[1], curr_direction[0]) - std::atan2(base_direction[1], base_direction[0]);

    if(yaw > M_PI)
    {
        yaw -= 2*M_PI;
    }
    if(yaw < (-M_PI))
    {
        yaw += 2*M_PI;
    }

    yaw = yaw / M_PI * 180.0;

    return yaw;
}

double bcd_planner::ComputeDistance(const Point2D& start, const Point2D& end, double meters_per_pix){
    double dist = std::sqrt(std::pow((end.x-start.x),2)+std::pow((end.y-start.y),2));
    dist = dist * meters_per_pix;
    return dist;
}

std::vector<NavigationMessage> bcd_planner::GetNavigationMessage(const Eigen::Vector2d& curr_direction, std::deque<Point2D> pos_path, double meters_per_pix){
     // initialization
    Eigen::Vector2d global_base_direction = {0, -1}; // {x, y}
    Eigen::Vector2d local_base_direction = curr_direction;

    Eigen::Vector2d curr_local_direction;
    Eigen::Vector2d curr_global_direction;

    NavigationMessage message;
    std::vector<NavigationMessage> message_queue;

    double distance = 0.0;
    double step_distance = 0.0;

    double prev_global_yaw = ComputeYaw(curr_direction, global_base_direction);

    double curr_global_yaw = 0.0;
    double curr_local_yaw = 0.0;

    message.SetGlobalYaw(DBL_MAX);
    message.SetLocalYaw(DBL_MAX);

    for(int i = 0; i < pos_path.size()-1; i++)
    {
        if(pos_path[i+1]==pos_path[i])
        {
            continue;
        }
        else
        {
            curr_local_direction = {pos_path[i+1].x-pos_path[i].x, pos_path[i+1].y-pos_path[i].y};
            curr_local_direction.normalize();

            curr_global_yaw = ComputeYaw(curr_local_direction, global_base_direction);
            curr_local_yaw = ComputeYaw(curr_local_direction, local_base_direction);

            if(message.GetGlobalYaw()==DBL_MAX) // initialization
            {
                message.SetGlobalYaw(curr_global_yaw);
            }

            if(message.GetLocalYaw()==DBL_MAX) // initialization
            {
                message.SetLocalYaw(curr_local_yaw);
            }

            if(curr_global_yaw == prev_global_yaw)
            {
                step_distance = ComputeDistance(pos_path[i+1], pos_path[i], meters_per_pix);
                distance += step_distance;
            }
            else
            {
                message.SetDistance(distance);
                message_queue.emplace_back(message);

                message.Reset();
                message.SetGlobalYaw(curr_global_yaw);
                message.SetLocalYaw(curr_local_yaw);

                distance = 0.0;
                step_distance = ComputeDistance(pos_path[i+1], pos_path[i], meters_per_pix);
                distance += step_distance;
            }
            prev_global_yaw = curr_global_yaw;

            local_base_direction = curr_local_direction;
        }
    }

    message.SetDistance(distance);
    message_queue.emplace_back(message);

    return message_queue;
}

int bcd_planner::GetFrontDirection(const Point2D& curr_pos, const Point2D& next_pos){
    int delta_x = next_pos.x - curr_pos.x;
    int delta_y = next_pos.y - curr_pos.y;

    if(delta_y < 0)
    {
        if(delta_x == 0)
        {
            return UP;
        }
        if(delta_x < 0)
        {
            return UPLEFT;
        }
        if(delta_x > 0)
        {
            return UPRIGHT;
        }
    }

    if(delta_y > 0)
    {
        if(delta_x == 0)
        {
            return DOWN;
        }
        if(delta_x < 0)
        {
            return DOWNLEFT;
        }
        if(delta_x > 0)
        {
            return DOWNRIGHT;
        }
    }

    if(delta_y == 0)
    {
        if(delta_x == 0)
        {
            return CENTER;
        }
        if(delta_x < 0)
        {
            return LEFT;
        }
        if(delta_x > 0)
        {
            return RIGHT;
        }
    }
}

int bcd_planner::GetBackDirection(int front_direction){
    if(front_direction + 4 >= map_directions.size())
    {
        int index_offset = front_direction + 4 - map_directions.size();
        return map_directions[index_offset];
    }
    else
    {
        return map_directions[front_direction+4];
    }
}

int bcd_planner::GetLeftDirection(int front_direction){
    if(front_direction - 2 < 0)
    {
        int index_offset = 2 - front_direction;
        return map_directions[map_directions.size()-index_offset];
    }
    else
    {
        return map_directions[front_direction-2];
    }
}

int bcd_planner::GetRightDirection(int front_direction){
    if(front_direction + 2 >= map_directions.size())
    {
        int index_offset = front_direction + 2 - map_directions.size();
        return map_directions[index_offset];
    }
    else
    {
        return map_directions[front_direction+2];
    }
}

std::vector<int> bcd_planner::GetFrontDirectionCandidates(int front_direction){
    std::vector<int> front_directions;

    if(front_direction - 2 < 0)
    {
        int index_offset = 2 - front_direction;
        front_directions.emplace_back(map_directions[map_directions.size()-index_offset]);
    }
    else
    {
        front_directions.emplace_back(map_directions[front_direction-2]);
    }

    if(front_direction - 1 < 0)
    {
        int index_offset = 1 - front_direction;
        front_directions.emplace_back(map_directions[map_directions.size()-index_offset]);
    }
    else
    {
        front_directions.emplace_back(map_directions[front_direction-1]);
    }

    front_directions.emplace_back(map_directions[front_direction]);


    if(front_direction + 1 >= map_directions.size())
    {
        int index_offset = front_direction + 1 - map_directions.size();
        front_directions.emplace_back(map_directions[index_offset]);
    }
    else
    {
        front_directions.emplace_back(map_directions[front_direction+1]);
    }

    if(front_direction + 2 >= map_directions.size())
    {
        int index_offset = front_direction + 2 - map_directions.size();
        front_directions.emplace_back(map_directions[index_offset]);
    }
    else
    {
        front_directions.emplace_back(map_directions[front_direction+2]);
    }

    return front_directions;

 }


std::vector<int> bcd_planner::GetBackDirectionCandidates(int front_direction){
    std::vector<int> back_directions;
    int first_direction = GetRightDirection(front_direction);

    for(int i = 0; i <= 4; i++)
    {
        if(first_direction + i >= map_directions.size())
        {
            int index_offset = first_direction + i - map_directions.size();
            back_directions.emplace_back(map_directions[index_offset]);
        }
        else
        {
            back_directions.emplace_back(map_directions[first_direction + i]);
        }
    }

    return back_directions;

 }

std::vector<int> bcd_planner::GetLeftDirectionCandidates(int front_direction){
    std::vector<int> left_directions;

    for(int i = 4; i >= 0; i--)
    {
        if(front_direction - i < 0)
        {
            int index_offset = i - front_direction;
            left_directions.emplace_back(map_directions[map_directions.size()-index_offset]);
        }
        else
        {
            left_directions.emplace_back(map_directions[front_direction-i]);
        }
    }

    return left_directions;

}

std::vector<int> bcd_planner::GetRightDirectionCandidates(int front_direction){
    std::vector<int> right_directions;
    for(int i = 0; i <= 4; i++)
    {
        if(front_direction + i >= map_directions.size())
        {
            int index_offset = front_direction + i - map_directions.size();
            right_directions.emplace_back(map_directions[index_offset]);
        }
        else
        {
            right_directions.emplace_back(map_directions[front_direction + i]);
        }
    }

    return right_directions;
}

Point2D bcd_planner::GetNextPosition(const Point2D& curr_pos,  int direction, int steps){
    Point2D next_position;


    switch (direction)
    {
        case 0:                                              //UP
            next_position.x = int(curr_pos.x);
            next_position.y = int(curr_pos.y - steps);
            return next_position;
        case 1:                                             //UPRIGHT 
            next_position.x = int(curr_pos.x + steps);
            next_position.y = int(curr_pos.y - steps);
            return next_position; 
        case 2:                                             //RIGHT
            next_position.x = int(curr_pos.x + steps);
            next_position.y = int(curr_pos.y);
            return next_position;
        case 3:                                             //DOWNRIGHT
            next_position.x = int(curr_pos.x + steps);
            next_position.y = int(curr_pos.y + steps);
            return next_position;
        case 4:                                            //DOWN 
            next_position.x = int(curr_pos.x);
            next_position.y = int(curr_pos.y + steps);
            return next_position;
        case 5:                                             //DOWNLEFT
            next_position.x = int(curr_pos.x - steps);
            next_position.y = int(curr_pos.y + steps);
            return next_position;
        case 6:                                             //LEFT
            next_position.x = int(curr_pos.x - steps);
            next_position.y = int(curr_pos.y);
            return next_position;
        case 7:                                            //UPLEFT
            next_position.x = int(curr_pos.x - steps);
            next_position.y = int(curr_pos.y - steps);
            return next_position;
        case 8:                                            //CENTER
            next_position.x = int(curr_pos.x);
            next_position.y = int(curr_pos.y);
            return next_position;
        default:
            return next_position;
    }
}

bool bcd_planner::CollisionOccurs(const cv::Mat& map, const Point2D& curr_pos, int detect_direction, int robot_radius){
    int obstacle_dist = INT_MAX;
    Point2D ray_pos;

    for(int i = 1; i <= (robot_radius+1); i++)
    {
        ray_pos = GetNextPosition(curr_pos, detect_direction, i);

        if(ray_pos.x < 0 || ray_pos.y < 0 || ray_pos.x >= map.cols || ray_pos.y >= map.rows)
        {
            break;
        }

        if(map.at<cv::Vec3b>(ray_pos.y, ray_pos.x) == cv::Vec3b(255,255,255))
        {
            obstacle_dist = i;
            break;
        }
    }

    return (obstacle_dist == (robot_radius+1));
}

bool bcd_planner::WalkAlongObstacle(const cv::Mat& map,      const Point2D& obstacle_origin,               const Point2D& contouring_origin,
                       int detecting_direction, const std::vector<int>& direction_candidates,
                       Point2D& curr_pos,       int first_turning_direction,                  int second_turning_direction,
                       Polygon& obstacle,       Polygon& new_obstacle,                        bool& isObstacleCompleted,
                       std::deque<Point2D>& contouring_path,                                  int robot_radius){

    bool turning = false;
    Point2D last_curr_pos = curr_pos;
    Point2D next_pos;
    Point2D obstacle_point;

    while(!turning)
    {
        for (auto direction: direction_candidates)
        {
            next_pos = GetNextPosition(curr_pos, direction, 1);
            if(next_pos.x < 0 || next_pos.y < 0 || next_pos.x >= map.cols || next_pos.y >= map.rows)
            {
                continue;
            }

            if (CollisionOccurs(map, next_pos, detecting_direction, robot_radius))
            {
                contouring_path.emplace_back(next_pos);
                curr_pos = next_pos;
                obstacle_point = GetNextPosition(next_pos, detecting_direction, robot_radius+1);
                if(obstacle_point.x==obstacle_origin.x && obstacle_point.y == obstacle_origin.y)
                {
                    if(!isObstacleCompleted)
                    {
                        obstacle.assign(new_obstacle.begin(), new_obstacle.end());
                        isObstacleCompleted = true;
                    }
                }
                new_obstacle.emplace_back(obstacle_point);
                break;
            }
        }
        if(curr_pos.x==last_curr_pos.x&&curr_pos.y==last_curr_pos.y)
        {
            turning = true;
        }
        else
        {
            last_curr_pos = curr_pos;
        }

        if(std::find(contouring_path.begin(), (contouring_path.end()-1), next_pos)!=(contouring_path.end()-1)
        &&contouring_path.size()>1
        &&isObstacleCompleted)
        {
            return false;
        }
    }
    for(int i = 1; i <= (robot_radius+1); i++)
    {
        next_pos = GetNextPosition(curr_pos, first_turning_direction, 1);
        contouring_path.emplace_back(next_pos);
        curr_pos = next_pos;

        if(std::find(contouring_path.begin(), (contouring_path.end()-1), next_pos)!=(contouring_path.end()-1)
        &&contouring_path.size()>1
        &&isObstacleCompleted)
        {
            return false;
        }
    }
    for(int i = 1; i <= (robot_radius+1); i++)
    {
        next_pos = GetNextPosition(curr_pos, second_turning_direction, 1);
        contouring_path.emplace_back(next_pos);
        curr_pos = next_pos;

        if(std::find(contouring_path.begin(), (contouring_path.end()-1), next_pos)!=(contouring_path.end()-1)
        &&contouring_path.size()>1
        &&isObstacleCompleted)
        {
            return false;
        }
    }
    return true;

}

Polygon bcd_planner::GetNewObstacle(const cv::Mat& map, Point2D origin, int front_direction, std::deque<Point2D>& contouring_path, int robot_radius){
        contouring_path.emplace_back(origin);

    Point2D curr_pos = origin;

    Point2D obstacle_point;
    Polygon new_obstacle;
    Polygon obstacle;

    bool isObstacleCompleted = false;

    int left_direction = GetLeftDirection(front_direction);
    int right_direction = GetRightDirection(front_direction);
    int back_direction = GetBackDirection(front_direction);

    std::deque<int> direcition_list = {right_direction, front_direction, left_direction, back_direction};

    std::vector<int> right_direction_candidates = GetRightDirectionCandidates(front_direction);
    std::vector<int> front_direction_candidates = GetFrontDirectionCandidates(front_direction);
    std::vector<int> left_direction_candidates = GetLeftDirectionCandidates(front_direction);
    std::vector<int> back_direction_candidates = GetBackDirectionCandidates(front_direction);

    std::deque<std::vector<int>> direction_candidates_list = {right_direction_candidates, front_direction_candidates, left_direction_candidates, back_direction_candidates};

    obstacle_point = GetNextPosition(origin, front_direction, robot_radius+1);
    new_obstacle.emplace_back(obstacle_point);

    Point2D obstacle_origin = new_obstacle.front();

    int detecting_direction;
    int first_turning_direction;
    int second_turning_direction;
    int temp_direction;

    bool keepContouring = true;
    std::vector<int> direction_candidates;
    std::vector<int> temp_direction_candidates;


    while(keepContouring)
    {
        direction_candidates = direction_candidates_list[0];
        first_turning_direction = direcition_list[0];
        second_turning_direction = direcition_list[1];
        detecting_direction = direcition_list[1];

        keepContouring = WalkAlongObstacle(map, obstacle_origin, origin, detecting_direction, direction_candidates, curr_pos, first_turning_direction, second_turning_direction
                , obstacle, new_obstacle, isObstacleCompleted, contouring_path, robot_radius);

        temp_direction = direcition_list.front();
        direcition_list.pop_front();
        direcition_list.emplace_back(temp_direction);

        temp_direction_candidates = direction_candidates_list.front();
        direction_candidates_list.pop_front();
        direction_candidates_list.emplace_back(temp_direction_candidates);
    }

    contouring_path.pop_front();

    return obstacle;
}

int bcd_planner::GetCleaningDirection(const CellNode& cell, Point2D exit){
    std::vector<Point2D> corner_points = ComputeCellCornerPoints(cell);

    double dist_to_left = std::abs(exit.x - corner_points[TOPLEFT].x);
    double dist_to_right = std::abs(exit.x = corner_points[TOPRIGHT].x);

    if(dist_to_left >= dist_to_right)
    {
        return RIGHT;
    }
    else
    {
        return LEFT;
    }
}

std::deque<std::deque<Point2D>> bcd_planner::LocalReplanning(cv::Mat& map, CellNode outer_cell, const PolygonList& obstacles, const Point2D& curr_pos, std::vector<CellNode>& curr_cell_graph, int cleaning_direction, int robot_radius, bool visualize_cells, bool visualize_path){
        //TODO: 边界判断
    int start_x = INT_MAX;
    int end_x = INT_MAX;

    if(cleaning_direction == LEFT)
    {
        start_x = outer_cell.ceiling.front().x;

        if(curr_pos.x + 2*(robot_radius + 1) <= outer_cell.ceiling.back().x)
        {
            end_x =  curr_pos.x + 2*(robot_radius + 1);
        }
        else
        {
            end_x = outer_cell.ceiling.back().x;
        }
    }
    if(cleaning_direction == RIGHT)
    {
        end_x = outer_cell.ceiling.back().x;

        if(curr_pos.x - 2*(robot_radius + 1) >= outer_cell.ceiling.front().x)
        {
            start_x = curr_pos.x - 2*(robot_radius + 1);
        }
        else
        {
            start_x = outer_cell.ceiling.front().x;
        }
    }
    int outer_cell_start_index_offset = start_x - outer_cell.ceiling.front().x;
    int outer_cell_end_index_offset = end_x - outer_cell.ceiling.front().x;

    CellNode inner_cell;
    for(int i = outer_cell_start_index_offset; i <= outer_cell_end_index_offset; i++)
    {
        inner_cell.ceiling.emplace_back(outer_cell.ceiling[i]);
        inner_cell.floor.emplace_back(outer_cell.floor[i]);
    }

//    curr_cell_graph = GenerateCells(map, inner_cell, obstacles);   // 这里需要改
    std::deque<std::deque<Point2D>> replanning_path = StaticPathPlanning(map, curr_cell_graph, curr_pos, robot_radius, visualize_cells, visualize_path);

    return replanning_path;

}

std::deque<Point2D> bcd_planner::DynamicPathPlanning(cv::Mat& map, const std::vector<CellNode>& global_cell_graph, std::deque<std::deque<Point2D>> global_path, int robot_radius, bool returning_home, bool visualize_path, int color_repeats){
    std::deque<Point2D> dynamic_path;

    std::deque<std::deque<Point2D>> curr_path;
    std::deque<Point2D> curr_sub_path;
    std::deque<Point2D> contouring_path;

    std::deque<std::deque<Point2D>> replanning_path;
    std::deque<std::deque<Point2D>> remaining_curr_path;

    std::deque<Point2D> linking_path;

    Point2D curr_pos;
    Point2D next_pos;
    Point2D curr_exit;

    int front_direction;
    int cleaning_direction;

    Polygon new_obstacle;
//    Polygon temp_new_obstacle;

    int curr_cell_index;
    CellNode curr_cell;

    std::vector<CellNode> curr_cell_graph;

    PolygonList overall_obstacles;
    PolygonList curr_obstacles;
    std::vector<cv::Point> visited_obstacle_contour;
    std::vector<std::vector<cv::Point>> visited_obstacle_contours;

    std::vector<std::deque<std::deque<Point2D>>> unvisited_paths = {global_path};
    std::vector<std::vector<CellNode>> cell_graph_list = {global_cell_graph};
    std::vector<Point2D> exit_list = {global_path.back().back()};

    cv::Mat vismap = map.clone();
    std::deque<cv::Scalar> JetColorMap;
    InitializeColorMap(JetColorMap, color_repeats);
    if(visualize_path)
    {
        cv::namedWindow("map", cv::WINDOW_NORMAL);
        cv::imshow("map", vismap);
    }


    while(!unvisited_paths.empty() && !cell_graph_list.empty())
    {
        curr_path = unvisited_paths.back();
        curr_cell_graph = cell_graph_list.back();

        for(int i = 0; i < curr_path.size(); i++)
        {
            curr_sub_path.assign(curr_path[i].begin(), curr_path[i].end());

            for(int j = 0; j < curr_sub_path.size()-1; j++)
            {
                curr_pos = curr_sub_path[j];
                next_pos = curr_sub_path[j+1];
                dynamic_path.emplace_back(curr_pos);

                if(visualize_path)
                {
                    vismap.at<cv::Vec3b>(curr_pos.y, curr_pos.x)=cv::Vec3b(uchar(JetColorMap.front()[0]),uchar(JetColorMap.front()[1]),uchar(JetColorMap.front()[2]));
                    UpdateColorMap(JetColorMap);
                    cv::imshow("map", vismap);
                    cv::waitKey(1);
                }

                front_direction = GetFrontDirection(curr_pos, next_pos);
                if(CollisionOccurs(map, curr_pos, front_direction, robot_radius))
                {
                    new_obstacle = GetNewObstacle(map, curr_pos, front_direction, contouring_path, robot_radius);
//                    new_obstacle = GetSingleContouringArea(map, temp_new_obstacle, robot_radius);
                    overall_obstacles.emplace_back(new_obstacle);

                    // for debugging
//                    for(int i = 0; i < new_obstacle.size(); i++)
//                    {
//                        std::cout<<"x:"<< new_obstacle[i].x <<", y:"<< new_obstacle[i].y <<std::endl;
//                    }
//
//                    for(int i = 0; i < new_obstacle.size(); i++)
//                    {
//                        vismap.at<cv::Vec3b>(new_obstacle[i].y, new_obstacle[i].x)=cv::Vec3b(0, 255, 0);
//                    }
//                    for(int i = 0; i < contouring_path.size(); i++)
//                    {
//                        vismap.at<cv::Vec3b>(contouring_path[i].y, contouring_path[i].x)=cv::Vec3b(255, 0, 0);
//                    }
//                    cv::circle(map, cv::Point(contouring_path.front().x, contouring_path.front().y), 2, cv::Scalar(0, 255, 255), -1);
//                    cv::circle(map, cv::Point(contouring_path.back().x, contouring_path.back().y), 2, cv::Scalar(255, 0, 255), -1);
//                    PointTypeTest(map, new_obstacle);
//                    cv::imshow("map", vismap);
//                    cv::waitKey(0);

                    dynamic_path.insert(dynamic_path.end(), contouring_path.begin(), contouring_path.end());

                    if(visualize_path)
                    {
                        for(const auto& point : contouring_path)
                        {
                            vismap.at<cv::Vec3b>(point.y, point.x)=cv::Vec3b(uchar(JetColorMap.front()[0]),uchar(JetColorMap.front()[1]),uchar(JetColorMap.front()[2]));
                            UpdateColorMap(JetColorMap);
                            cv::imshow("map", vismap);
                            cv::waitKey(1);
                        }
                    }

                    contouring_path.clear();

                    visited_obstacle_contour.clear();
                    visited_obstacle_contours.clear();

                    for(const auto& point: new_obstacle)
                    {
                        visited_obstacle_contour.emplace_back(cv::Point(point.x,point.y));
                    }

                    visited_obstacle_contours.emplace_back(visited_obstacle_contour);

                    curr_cell_index = DetermineCellIndex(curr_cell_graph, curr_pos).front();
                    curr_cell = curr_cell_graph[curr_cell_index];
                    curr_obstacles={new_obstacle};
                    curr_exit = curr_sub_path.back();

                    // for debugging
//                    DrawCells(map, curr_cell, cv::Scalar(255, 0, 255));
//                    vismap.at<cv::Vec3b>(curr_exit.y, curr_exit.x)=cv::Vec3b(0,255,255);
//                    cv::imshow("map", vismap);
//                    cv::waitKey(0);

                    cleaning_direction = GetCleaningDirection(curr_cell, curr_exit);

                    replanning_path = LocalReplanning(map, curr_cell, curr_obstacles, dynamic_path.back(), curr_cell_graph, cleaning_direction, robot_radius, false, false); // 此处会更新curr_cell_graph
                    cv::fillPoly(map, visited_obstacle_contours, cv::Scalar(50, 50, 50));
                    cv::fillPoly(vismap, visited_obstacle_contours, cv::Scalar(50, 50, 50));

                    remaining_curr_path.assign(curr_path.begin()+i+1, curr_path.end());

                    goto UPDATING_REMAINING_PATHS;
                }
            }
        }

        if(dynamic_path.back().x != exit_list.back().x && dynamic_path.back().y != exit_list.back().y)
        {
            linking_path = ReturningPathPlanning(map, cell_graph_list.back(), dynamic_path.back(), exit_list.back(), robot_radius, false);
            dynamic_path.insert(dynamic_path.end(), linking_path.begin(), linking_path.end());

            if(visualize_path)
            {
                for(const auto& point : linking_path)
                {
                    vismap.at<cv::Vec3b>(point.y, point.x)=cv::Vec3b(uchar(JetColorMap.front()[0]),uchar(JetColorMap.front()[1]),uchar(JetColorMap.front()[2]));
                    UpdateColorMap(JetColorMap);
                    cv::imshow("map", vismap);
                    cv::waitKey(1);
                }
            }
        }

        exit_list.pop_back();
        unvisited_paths.pop_back();
        cell_graph_list.pop_back();
        continue;

        UPDATING_REMAINING_PATHS:
        exit_list.emplace_back(curr_exit);
        unvisited_paths.pop_back();
        unvisited_paths.emplace_back(remaining_curr_path);
        unvisited_paths.emplace_back(replanning_path);
        cell_graph_list.emplace_back(curr_cell_graph);
    }

    if(returning_home)
    {
        cv::Mat3b returning_map = cv::Mat3b(map.size(), CV_8U);
        returning_map.setTo(cv::Scalar(0, 0, 0));
        std::vector<cv::Point> returning_obstacle_contour;
        std::vector<std::vector<cv::Point>> returning_obstacle_contours;

        for(const auto& obstacle: overall_obstacles)
        {
            for(const auto& point : obstacle)
            {
                returning_obstacle_contour.emplace_back(cv::Point(point.x,point.y));
            }
            returning_obstacle_contours.emplace_back(returning_obstacle_contour);
            returning_obstacle_contour.clear();
        }

        cv::fillPoly(returning_map, returning_obstacle_contours, cv::Scalar(255, 255, 255));

        Polygon returning_map_border = ConstructDefaultWall(returning_map);
//        std::vector<CellNode> returning_cell_graph = GenerateCells(returning_map, returning_map_border, overall_obstacles);//这里需要改
        std::vector<CellNode> returning_cell_graph; //这里需要改

        // for debugging
//        for(auto cell:returning_cell_graph)
//        {
//            DrawCells(vismap, cell, cv::Scalar(0, 255, 255));
//            cv::imshow("map", vismap);
//            cv::waitKey(0);
//        }
        //

        for(auto cell : returning_cell_graph)
        {
            cell.isCleaned = true;
        }

        std::deque<Point2D> returning_path = ReturningPathPlanning(returning_map, returning_cell_graph, dynamic_path.back(), dynamic_path.front(), robot_radius, false);

        if(visualize_path)
        {
            for(const auto& point : returning_path)
            {
                vismap.at<cv::Vec3b>(point.y, point.x)=cv::Vec3b(250,250,250);
                cv::imshow("map", vismap);
                cv::waitKey(1);
            }
        }

        dynamic_path.insert(dynamic_path.end(), returning_path.begin(), returning_path.end());
    }

    if(visualize_path)
    {
        cv::waitKey(5000);
    }

    return dynamic_path;
}

void bcd_planner::MoveAsPathPlannedTest(cv::Mat& map, double meters_per_pix, const Point2D& start, const std::vector<NavigationMessage>& motion_commands){
    int pixs;
    Point2D begin = start, end;

    cv::namedWindow("original_map", cv::WINDOW_NORMAL);

    for(auto command:motion_commands)
    {
        pixs = int(command.GetDistance()/meters_per_pix);
        if(command.GetGlobalYaw()==0.0)
        {
            end = Point2D(begin.x, begin.y-pixs);
            cv::line(map, cv::Point(begin.x, begin.y), cv::Point(end.x, end.y), cv::Scalar(0, 0, 255));
            cv::imshow("original_map", map);
            cv::waitKey(100);
        }
        if(command.GetGlobalYaw()==180.0)
        {
            end = Point2D(begin.x, begin.y+pixs);
            cv::line(map, cv::Point(begin.x, begin.y), cv::Point(end.x, end.y), cv::Scalar(255, 0, 0));
            cv::imshow("original_map", map);
            cv::waitKey(100);
        }
        if(command.GetGlobalYaw()==90.0)
        {
            end = Point2D(begin.x+pixs, begin.y);
            cv::line(map, cv::Point(begin.x, begin.y), cv::Point(end.x, end.y), cv::Scalar(0, 255, 0));
            cv::imshow("original_map", map);
            cv::waitKey(100);
        }
        if(command.GetGlobalYaw()==-90.0)
        {
            end = Point2D(begin.x-pixs, begin.y);
            cv::line(map, cv::Point(begin.x, begin.y), cv::Point(end.x, end.y), cv::Scalar(0, 255, 255));
            cv::imshow("original_map", map);
            cv::waitKey(100);
        }
        begin = end;
    }
    cv::waitKey(0);

}

std::vector<std::vector<cv::Point>> bcd_planner::ConstructHandcraftedContours1(){
    std::vector<cv::Point> handcrafted_polygon_1_1 = {cv::Point(100,200), cv::Point(200,200), cv::Point(200,100), cv::Point(100,100)};
    std::vector<cv::Point> handcrafted_polygon_1_2 = {cv::Point(350,200), cv::Point(400,200), cv::Point(400,150), cv::Point(350,150)};
    std::vector<std::vector<cv::Point>> contours = {handcrafted_polygon_1_1, handcrafted_polygon_1_2};
    return contours;
}

std::vector<std::vector<cv::Point>> bcd_planner::ConstructHandcraftedContours2(){
    std::vector<cv::Point> handcrafted_polygon_2 = {cv::Point(100,500), cv::Point(200,500), cv::Point(200,400), cv::Point(400,400),
                                                    cv::Point(400,500), cv::Point(500,500), cv::Point(500,100), cv::Point(400,100),
                                                    cv::Point(400,200), cv::Point(200,200), cv::Point(200,100), cv::Point(100,100),
                                                    };
    std::vector<std::vector<cv::Point>> contours = {handcrafted_polygon_2};
    return contours;
}

std::vector<std::vector<cv::Point>> bcd_planner::ConstructHandcraftedContours3(){
    std::vector<cv::Point> handcrafted_polygon_3 = {cv::Point(100,100), cv::Point(100,500), cv::Point(150,500), cv::Point(150,150),
                                                    cv::Point(450,150), cv::Point(450,300), cv::Point(300,300), cv::Point(300,250),
                                                    cv::Point(350,250), cv::Point(350,200), cv::Point(250,200), cv::Point(250,350),
                                                    cv::Point(500,350), cv::Point(500,100)};
    std::vector<std::vector<cv::Point>> contours = {handcrafted_polygon_3};
    return contours;
}

std::vector<std::vector<cv::Point>> bcd_planner::ConstructHandcraftedContours4(){
     std::vector<cv::Point> handcrafted_polygon_4_1 = {cv::Point(20,20),  cv::Point(20,200), cv::Point(100,200),cv::Point(100,399),
                                                      cv::Point(20,399), cv::Point(20, 579),cv::Point(200,579),cv::Point(200,499),cv::Point(399,499),cv::Point(399,579),
                                                      cv::Point(579,579),cv::Point(579,399),cv::Point(499,399),cv::Point(499,200),cv::Point(579,200),cv::Point(579,20),
                                                      cv::Point(349,20), cv::Point(349,100),cv::Point(250,100),cv::Point(250,20)};
    std::vector<cv::Point> handcrafted_polygon_4_2 = {cv::Point(220,220),cv::Point(220,380),cv::Point(380,380),cv::Point(380,220)};
    std::vector<std::vector<cv::Point>> contours = {handcrafted_polygon_4_1, handcrafted_polygon_4_2};
    return contours;
}

std::vector<std::vector<cv::Point>> bcd_planner::ConstructHandcraftedContours5(){
    std::vector<cv::Point> handcrafted_polygon_5_1 = {cv::Point(125, 50), cv::Point(50, 125), cv::Point(125, 200), cv::Point(200, 125)};
    std::vector<cv::Point> handcrafted_polygon_5_2 = {cv::Point(80, 300), cv::Point(80, 400), cv::Point(160, 400), cv::Point(120, 350),
                                                      cv::Point(160, 300)};
    std::vector<cv::Point> handcrafted_polygon_5_3 = {cv::Point(100, 450), cv::Point(100, 550), cv::Point(140, 550), cv::Point(140, 450)};
    std::vector<cv::Point> handcrafted_polygon_5_4 = {cv::Point(300, 150), cv::Point(300, 250), cv::Point(400, 220), cv::Point(400, 180)};
    std::vector<std::vector<cv::Point>> contours = {handcrafted_polygon_5_1, handcrafted_polygon_5_2, handcrafted_polygon_5_3, handcrafted_polygon_5_4};
    return contours;
}

std::vector<std::vector<cv::Point>> bcd_planner::ConstructHandcraftedContours6(){
    std::vector<cv::Point> handcrafted_polygon_1 = {cv::Point(250,350), cv::Point(350,350), cv::Point(350,250), cv::Point(250,250)};
    std::vector<cv::Point> handcrafted_polygon_2 = {cv::Point(425,350), cv::Point(475,350), cv::Point(475,250), cv::Point(425,250)};
    std::vector<std::vector<cv::Point>> contours = {handcrafted_polygon_1,handcrafted_polygon_2};
    return contours;
}

void bcd_planner::CheckObstaclePointType(cv::Mat& map, const Polygon& obstacle){
    PolygonList obstacles = {obstacle};

    std::vector<Event> event_list = GenerateObstacleEventList(map, obstacles);

    for(auto event: event_list)
    {
        if(event.event_type == IN)
        {
            std::cout<<event.x<<", "<<event.y<<", IN"<<std::endl;
            cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 255, 0), -1);
        }
        if(event.event_type == IN_TOP)
        {
            std::cout<<event.x<<", "<<event.y<<", IN_TOP"<<std::endl;
            cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 255, 0), -1);
        }
        if(event.event_type == IN_BOTTOM)
        {
            std::cout<<event.x<<", "<<event.y<<", IN_BOTTOM"<<std::endl;
            cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 255, 0), -1);
        }
        if(event.event_type == OUT)
        {
            std::cout<<event.x<<", "<<event.y<<", OUT"<<std::endl;
            cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 0, 255), -1);
        }
        if(event.event_type == OUT_TOP)
        {
            std::cout<<event.x<<", "<<event.y<<", OUT_TOP"<<std::endl;
            cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 0, 255), -1);
        }
        if(event.event_type == OUT_BOTTOM)
        {
            std::cout<<event.x<<", "<<event.y<<", OUT_BOTTOM"<<std::endl;
            cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 0, 255), -1);
        }
        if(event.event_type == INNER_IN)
        {
            std::cout<<event.x<<", "<<event.y<<", INNER_IN"<<std::endl;
            cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 255, 0), -1);
        }
        if(event.event_type == INNER_IN_TOP)
        {
            std::cout<<event.x<<", "<<event.y<<", INNER_IN_TOP"<<std::endl;
            cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 255, 0), -1);
        }
        if(event.event_type == INNER_IN_BOTTOM)
        {
            std::cout<<event.x<<", "<<event.y<<", INNER_IN_BOTTOM"<<std::endl;
            cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 255, 0), -1);
        }
        if(event.event_type == INNER_OUT)
        {
            std::cout<<event.x<<", "<<event.y<<", INNER_OUT"<<std::endl;
            cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 0, 255), -1);
        }
        if(event.event_type == INNER_OUT_TOP)
        {
            std::cout<<event.x<<", "<<event.y<<", INNER_OUT_TOP"<<std::endl;
            cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 0, 255), -1);
        }
        if(event.event_type == INNER_OUT_BOTTOM)
        {
            std::cout<<event.x<<", "<<event.y<<", INNER_OUT_BOTTOM"<<std::endl;
            cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 0, 255), -1);
        }
        if(event.event_type == MIDDLE)
        {
            map.at<cv::Vec3b>(event.y, event.x) = cv::Vec3b(50, 50 ,50);
        }
        if(event.event_type == CEILING)
        {
            map.at<cv::Vec3b>(event.y, event.x) = cv::Vec3b(0, 255 ,255);
        }
        if(event.event_type == FLOOR)
        {
            map.at<cv::Vec3b>(event.y, event.x) = cv::Vec3b(255, 0 ,0);
        }
    }
}


void bcd_planner::CheckWallPointType(cv::Mat& map, const Polygon& wall){
    std::vector<Event> event_list = GenerateWallEventList(map, wall);
    for(auto event: event_list)
    {
        if(event.event_type == IN_EX)
        {
            std::cout<<event.x<<", "<<event.y<<", IN_EX"<<std::endl;
            cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 255, 0), -1);
        }
        if(event.event_type == IN_TOP_EX)
        {
            std::cout<<event.x<<", "<<event.y<<", IN_TOP_EX"<<std::endl;
            cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 255, 0), -1);
        }
        if(event.event_type == IN_BOTTOM_EX)
        {
            std::cout<<event.x<<", "<<event.y<<", IN_BOTTOM_EX"<<std::endl;
            cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 255, 0), -1);
        }
        if(event.event_type == OUT_EX)
        {
            std::cout<<event.x<<", "<<event.y<<", OUT_EX"<<std::endl;
            cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 0, 255), -1);
        }
        if(event.event_type == OUT_TOP_EX)
        {
            std::cout<<event.x<<", "<<event.y<<", OUT_TOP_EX"<<std::endl;
            cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 0, 255), -1);
        }
        if(event.event_type == OUT_BOTTOM_EX)
        {
            std::cout<<event.x<<", "<<event.y<<", OUT_BOTTOM_EX"<<std::endl;
            cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 0, 255), -1);
        }
        if(event.event_type == INNER_IN_EX)
        {
            std::cout<<event.x<<", "<<event.y<<", INNER_IN_EX"<<std::endl;
            cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 255, 0), -1);
        }
        if(event.event_type == INNER_IN_TOP_EX)
        {
            std::cout<<event.x<<", "<<event.y<<", INNER_IN_TOP_EX"<<std::endl;
            cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 255, 0), -1);
        }
        if(event.event_type == INNER_IN_BOTTOM_EX)
        {
            std::cout<<event.x<<", "<<event.y<<", INNER_IN_BOTTOM_EX"<<std::endl;
            cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 255, 0), -1);
        }
        if(event.event_type == INNER_OUT_EX)
        {
            std::cout<<event.x<<", "<<event.y<<", INNER_OUT_EX"<<std::endl;
            cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 0, 255), -1);
        }
        if(event.event_type == INNER_OUT_TOP_EX)
        {
            std::cout<<event.x<<", "<<event.y<<", INNER_OUT_TOP_EX"<<std::endl;
            cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 0, 255), -1);
        }
        if(event.event_type == INNER_OUT_BOTTOM_EX)
        {
            std::cout<<event.x<<", "<<event.y<<", INNER_OUT_BOTTOM_EX"<<std::endl;
            cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 0, 255), -1);
        }
        if(event.event_type == MIDDLE)
        {
            map.at<cv::Vec3b>(event.y, event.x) = cv::Vec3b(50, 50 ,50);
        }
        if(event.event_type == CEILING)
        {
            map.at<cv::Vec3b>(event.y, event.x) = cv::Vec3b(0, 255 ,255);
        }
        if(event.event_type == FLOOR)
        {
            map.at<cv::Vec3b>(event.y, event.x) = cv::Vec3b(255, 0 ,0);
        }
    }

}

void bcd_planner::CheckPointType(const cv::Mat& map, const Polygon& wall, const PolygonList& obstacles){
    cv::Mat vis_map = map.clone();
    cv::cvtColor(vis_map, vis_map, cv::COLOR_GRAY2BGR);

    cv::namedWindow("map", cv::WINDOW_NORMAL);

    for(const auto& obstacle:obstacles)
    {
        CheckObstaclePointType(vis_map, obstacle);
        cv::imshow("map", vis_map);
        cv::waitKey(0);
        std::cout<<std::endl;
    }

    CheckWallPointType(vis_map, wall);
    cv::imshow("map", vis_map);
    cv::waitKey(0);
}

void bcd_planner::CheckSlicelist(const std::deque<std::deque<Event>>& slice_list){
    std::deque<Event> slice;
    for(int i = 0; i < slice_list.size(); i++)
    {
        slice = FilterSlice(slice_list[i]);
        std::cout<<"slice "<<i<<": ";
        for(const auto& event : slice)
        {
            EventType type = event.event_type;
            switch (type)
            {
                case IN:
                    std::cout<<"IN; ";
                    break;
                case IN_TOP:
                    std::cout<<"IN_TOP; ";
                    break;
                case IN_BOTTOM:
                    std::cout<<"IN_BOTTOM; ";
                    break;
                case OUT:
                    std::cout<<"OUT; ";
                    break;
                case OUT_TOP:
                    std::cout<<"OUT_TOP; ";
                    break;
                case OUT_BOTTOM:
                    std::cout<<"OUT_BOTTOM; ";
                    break;
                case INNER_IN:
                    std::cout<<"INNER_IN; ";
                    break;
                case INNER_IN_TOP:
                    std::cout<<"INNER_IN_TOP; ";
                    break;
                case INNER_IN_BOTTOM:
                    std::cout<<"INNER_IN_BOTTOM; ";
                    break;
                case INNER_OUT:
                    std::cout<<"INNER_OUT; ";
                    break;
                case INNER_OUT_TOP:
                    std::cout<<"INNER_OUT_TOP; ";
                    break;
                case INNER_OUT_BOTTOM:
                    std::cout<<"INNER_OUT_BOTTOM; ";
                    break;
                case IN_EX:
                    std::cout<<"IN_EX; ";
                    break;
                case IN_TOP_EX:
                    std::cout<<"IN_TOP_EX; ";
                    break;
                case IN_BOTTOM_EX:
                    std::cout<<"IN_BOTTOM_EX; ";
                    break;
                case OUT_EX:
                    std::cout<<"OUT_EX; ";
                    break;
                case OUT_TOP_EX:
                    std::cout<<"OUT_TOP_EX; ";
                    break;
                case OUT_BOTTOM_EX:
                    std::cout<<"OUT_BOTTOM_EX; ";
                    break;
                case INNER_IN_EX:
                    std::cout<<"INNER_IN_EX; ";
                    break;
                case INNER_IN_TOP_EX:
                    std::cout<<"INNER_IN_TOP_EX; ";
                    break;
                case INNER_IN_BOTTOM_EX:
                    std::cout<<"INNER_IN_BOTTOM_EX; ";
                    break;
                case INNER_OUT_EX:
                    std::cout<<"INNER_OUT_EX; ";
                    break;
                case INNER_OUT_TOP_EX:
                    std::cout<<"INNER_OUT_TOP_EX; ";
                    break;
                case INNER_OUT_BOTTOM_EX:
                    std::cout<<"INNER_OUT_BOTTOM_EX; ";
                    break;
                case MIDDLE:
                    std::cout<<"MIDDLE; ";
                    break;
                case CEILING:
                    std::cout<<"CEILING; ";
                    break;
                case FLOOR:
                    std::cout<<"FLOOR; ";
                    break;
                case UNALLOCATED:
                    std::cout<<"UNALLOCATED; ";
                    break;
            }
        }
        std::cout<<std::endl;
    }
}

void bcd_planner::CheckExtractedContours(const cv::Mat& map, const std::vector<std::vector<cv::Point>>& contours){
    cv::namedWindow("map", cv::WINDOW_NORMAL);

    cv::Mat3b canvas = cv::Mat3b(map.size(), CV_8U);
    canvas.setTo(cv::Scalar(0,0,0));

    for(int i = 0; i<=contours.size()-1; i++)
    {
        cv::drawContours(canvas, contours, i, cv::Scalar(255, 0, 0));
        cv::imshow("map", canvas);
        cv::waitKey(0);
    }
}

void bcd_planner::CheckGeneratedCells(const cv::Mat& map, const std::vector<CellNode>& cell_graph){
    cv::Mat vis_map = map.clone();
    cv::cvtColor(vis_map, vis_map, cv::COLOR_GRAY2BGR);

    cv::namedWindow("map", cv::WINDOW_NORMAL);

    for(const auto& cell : cell_graph)
    {
        DrawCells(vis_map, cell, cv::Scalar(255, 0, 255));
        cv::imshow("map", vis_map);
        cv::waitKey(0);
    }
}

void bcd_planner::CheckPathNodes(const std::deque<std::deque<Point2D>>& path){
    for(const auto& subpath : path)
    {
        for(const auto& point : subpath)
        {
            std::cout<<point.x<<", "<<point.y<<std::endl;
        }
        std::cout<<std::endl;
    }
 }

void bcd_planner::CheckPathConsistency(const std::deque<Point2D>& path){
    int breakpoints = 0;
    int duplicates = 0;

    for(int i = 1; i < path.size(); i++)
    {
        if(std::abs(path[WrappedIndex((i-1),path.size())].x-path[i].x)>1||std::abs(path[WrappedIndex((i-1),path.size())].y-path[i].y)>1)
        {
            breakpoints++;
            std::cout<<"break points :"<<path[WrappedIndex((i-1),path.size())].x<<", "<<path[WrappedIndex((i-1),path.size())].y
                     <<"---->"<<path[i].x<<", "<<path[i].y<<std::endl;
        }
        if(path[WrappedIndex((i-1),path.size())]==path[i])
        {
            duplicates++;
        }

    }
    std::cout<<"breakpoints: "<<breakpoints<<std::endl;
    std::cout<<"duplicates: "<<duplicates<<std::endl;
 }

void bcd_planner::CheckMotionCommands(const std::vector<NavigationMessage>& navigation_messages){
    double dist = 0.0, global_yaw = 0.0, local_yaw = 0.0;
    for(auto message : navigation_messages)
    {
        message.GetMotion(dist, global_yaw, local_yaw);
        std::cout<<"globally rotate "<<global_yaw<<" degree(locally rotate "<<local_yaw<<" degree) and go forward for "<<dist<<" m."<<std::endl;
    }
}

void bcd_planner::StaticPathPlanningExample1(){
    double meters_per_pix = 0.02;
    double robot_size_in_meters = 0.15;

    int robot_radius = ComputeRobotRadius(meters_per_pix, robot_size_in_meters);

    cv::Mat1b map = ReadMap("../map.png");
    map = PreprocessMap(map);

    std::vector<std::vector<cv::Point>> obstacle_contours;
    std::vector<std::vector<cv::Point>> wall_contours;
    ExtractContours(map, wall_contours, obstacle_contours, robot_radius);

    Polygon wall = ConstructWall(map, wall_contours.front());
    PolygonList obstacles = ConstructObstacles(map, obstacle_contours);

    std::vector<CellNode> cell_graph = ConstructCellGraph(map, wall_contours, obstacle_contours, wall, obstacles);

    Point2D start = Point2D(map.cols/2, map.rows/2);
    std::deque<std::deque<Point2D>> original_planning_path = StaticPathPlanning(map, cell_graph, start, robot_radius, false, false);

    std::deque<Point2D> path = FilterTrajectory(original_planning_path);
    CheckPathConsistency(path);

    VisualizeTrajectory(map, path, robot_radius, PATH_MODE);

    Eigen::Vector2d curr_direction = {0, -1};
    std::vector<NavigationMessage> messages = GetNavigationMessage(curr_direction, path, meters_per_pix);
    CheckMotionCommands(messages);
}

void bcd_planner::StaticPathPlanningExample2(){
    int robot_radius = 5;
    cv::Mat1b map = ReadMap("../complicate_map.png");
    map = PreprocessMap(map);

    std::vector<std::vector<cv::Point>> wall_contours;
    std::vector<std::vector<cv::Point>> obstacle_contours;
    ExtractContours(map, wall_contours, obstacle_contours);

    Polygon wall = ConstructWall(map, wall_contours.front());
    PolygonList obstacles = ConstructObstacles(map, obstacle_contours);

    std::vector<CellNode> cell_graph = ConstructCellGraph(map, wall_contours, obstacle_contours, wall, obstacles);

    Point2D start = cell_graph.front().ceiling.front();
    std::deque<std::deque<Point2D>> original_planning_path = StaticPathPlanning(map, cell_graph, start, robot_radius, false, false);

    std::deque<Point2D> path = FilterTrajectory(original_planning_path);
    CheckPathConsistency(path);

    int time_interval = 1;
    VisualizeTrajectory(map, path, robot_radius, PATH_MODE, time_interval);

}

void bcd_planner::StaticPathPlanningExample3(){
    int robot_radius = 5;

    cv::Mat1b map = cv::Mat1b(cv::Size(500, 500), CV_8U);                               
    map.setTo(0);

    std::vector<std::vector<cv::Point>> contours = ConstructHandcraftedContours1();      
    cv::fillPoly(map, contours, 255);                                                     

    std::vector<std::vector<cv::Point>> obstacle_contours;
    std::vector<std::vector<cv::Point>> wall_contours;
    ExtractContours(map, wall_contours, obstacle_contours);
    CheckExtractedContours(map, wall_contours);
    CheckExtractedContours(map, obstacle_contours);

    PolygonList obstacles = ConstructObstacles(map, obstacle_contours);
    Polygon wall = ConstructWall(map, wall_contours.front());

    std::vector<CellNode> cell_graph = ConstructCellGraph(map, wall_contours, obstacle_contours, wall, obstacles);
    CheckPointType(map, wall ,obstacles);
    CheckGeneratedCells(map, cell_graph);

    Point2D start = cell_graph.front().ceiling.front();
    std::deque<std::deque<Point2D>> original_planning_path = StaticPathPlanning(map, cell_graph, start, robot_radius, false, false);
    CheckPathNodes(original_planning_path);

    std::deque<Point2D> path = FilterTrajectory(original_planning_path);
    CheckPathConsistency(path);

    int time_interval = 1;
    VisualizeTrajectory(map, path, robot_radius, PATH_MODE, time_interval);

}

void bcd_planner::StaticPathPlanningExample4(){
    int robot_radius = 5;

    cv::Mat1b map = cv::Mat1b(cv::Size(600, 600), CV_8U);
    map.setTo(0);
    
    std::vector<std::vector<cv::Point>> contours = ConstructHandcraftedContours2();
    cv::fillPoly(map, contours, 255);     //用墙进行隔断
    std::vector<std::vector<cv::Point>> obs =  ConstructHandcraftedContours6(); //add obs
    cv::fillPoly(map, obs , 0);
    //inflation(map,robot_radius);


    std::vector<std::vector<cv::Point>> obstacle_contours;
    std::vector<std::vector<cv::Point>> wall_contours;
    ExtractContours(map, wall_contours, obstacle_contours);
    // CheckExtractedContours(map, wall_contours);
    // CheckExtractedContours(map, obstacle_contours);

    PolygonList obstacles = ConstructObstacles(map, obstacle_contours);
    Polygon wall = ConstructWall(map, wall_contours.front());

    cv::Mat3b map_ = cv::Mat3b(map.size());
    map_.setTo(cv::Scalar(0, 0, 0));

    cv::fillPoly(map_, wall_contours, cv::Scalar(255, 255, 255));
    cv::fillPoly(map_, obstacle_contours, cv::Scalar(0, 0, 0));

    std::vector<Event> wall_event_list = GenerateWallEventList(map_, wall);   
                                                                    //根据wall polygon中的每个点的event类型来生成wall_event_list
    std::vector<Event> obstacle_event_list = GenerateObstacleEventList(map_, obstacles);                 //根据obstacle_event_list中的每个点的event类型来生成obstac_event_list
    std::deque<std::deque<Event>> slice_list = SliceListGenerator(wall_event_list, obstacle_event_list);
    CheckSlicelist(slice_list);

    std::vector<CellNode> cell_graph;
    std::vector<int> cell_index_slice;
    std::vector<int> original_cell_index_slice;
    ExecuteCellDecomposition(cell_graph, cell_index_slice, original_cell_index_slice, slice_list);
    CheckPointType(map, wall ,obstacles);
    CheckGeneratedCells(map, cell_graph);

    Point2D start = cell_graph.front().ceiling.front();
    std::deque<std::deque<Point2D>> original_planning_path = StaticPathPlanning(map, cell_graph, start, robot_radius, false, false);
    CheckPathNodes(original_planning_path);

    std::deque<Point2D> path = FilterTrajectory(original_planning_path);
    CheckPathConsistency(path);

    int time_interval = 1;
    VisualizeTrajectory(map, path, robot_radius, PATH_MODE, time_interval);

}

void bcd_planner::StaticPathPlanningExample5(){
    int robot_radius = 5;

    cv::Mat1b map = cv::Mat1b(cv::Size(600, 600), CV_8U);
    map.setTo(255);

    std::vector<std::vector<cv::Point>> contours = ConstructHandcraftedContours3();
    cv::fillPoly(map, contours, 0);

    std::vector<std::vector<cv::Point>> obstacle_contours;
    std::vector<std::vector<cv::Point>> wall_contours;
    ExtractContours(map, wall_contours, obstacle_contours);
    CheckExtractedContours(map, wall_contours);
    CheckExtractedContours(map, obstacle_contours);

    PolygonList obstacles = ConstructObstacles(map, obstacle_contours);
    Polygon wall = ConstructWall(map, wall_contours.front());

    std::vector<CellNode> cell_graph = ConstructCellGraph(map, wall_contours, obstacle_contours, wall, obstacles);
    CheckPointType(map, wall ,obstacles);
    CheckGeneratedCells(map, cell_graph);

    Point2D start = cell_graph.front().ceiling.front();
    std::deque<std::deque<Point2D>> original_planning_path = StaticPathPlanning(map, cell_graph, start, robot_radius, false, false);
    CheckPathNodes(original_planning_path);

    std::deque<Point2D> path = FilterTrajectory(original_planning_path);
    CheckPathConsistency(path);

    int time_interval = 1;
    VisualizeTrajectory(map, path, robot_radius, PATH_MODE, time_interval);

}

void bcd_planner::StaticPathPlanningExample6(){
    int robot_radius = 5;

    cv::Mat1b map = cv::Mat1b(cv::Size(600, 600), CV_8U);
    map.setTo(0);

    std::vector<std::vector<cv::Point>> contours = ConstructHandcraftedContours4();
    std::vector<std::vector<cv::Point>> external_contours = {contours.front()};
    std::vector<std::vector<cv::Point>> inner_contours = {contours.back()};
    cv::fillPoly(map, external_contours, 255);
    cv::fillPoly(map, inner_contours, 0);

    std::vector<std::vector<cv::Point>> obstacle_contours;
    std::vector<std::vector<cv::Point>> wall_contours;
    ExtractContours(map, wall_contours, obstacle_contours);
    CheckExtractedContours(map, wall_contours);
    CheckExtractedContours(map, obstacle_contours);

    PolygonList obstacles = ConstructObstacles(map, obstacle_contours);
    Polygon wall = ConstructWall(map, wall_contours.front());

    std::vector<CellNode> cell_graph = ConstructCellGraph(map, wall_contours, obstacle_contours, wall, obstacles);
    CheckPointType(map, wall ,obstacles);
    CheckGeneratedCells(map, cell_graph);

    Point2D start = cell_graph.front().ceiling.front();
    std::deque<std::deque<Point2D>> original_planning_path = StaticPathPlanning(map, cell_graph, start, robot_radius, false, false);
    CheckPathNodes(original_planning_path);

    std::deque<Point2D> path = FilterTrajectory(original_planning_path);
    CheckPathConsistency(path);

    int time_interval = 1;
    VisualizeTrajectory(map, path, robot_radius, PATH_MODE, time_interval);

}

void bcd_planner::TestAllExamples(){
     //StaticPathPlanningExample1();

    //StaticPathPlanningExample2();

    //StaticPathPlanningExample3();

    StaticPathPlanningExample4();

    // StaticPathPlanningExample5();

    // StaticPathPlanningExample6();

}

void bcd_planner::init(){
    t_map_world.setIdentity();
    Vector3d x_axis;  x_axis<<1.0, 0.0, 0.0;
    Vector3d y_axis;  y_axis<<0.0,-1.0, 0.0;
    Vector3d z_axis;  z_axis<<0.0, 0.0, 1.0;
    Vector3d pos;     pos<<   -15, 15,  0.0;
    t_map_world.block<3,1>(0,0) = x_axis;
    t_map_world.block<3,1>(0,1) = y_axis;
    t_map_world.block<3,1>(0,2) = z_axis;
    t_map_world.block<3,1>(0,3) = pos;

    resolution = 0.05;  
    inv_resolution = 1.0/resolution;
    num_of_colums = 600;
    num_of_rows = 600;
}

Vector2i bcd_planner::world2map(double x,double y){
    Vector4d pos_in_world;  pos_in_world<<x,y,0,1.0;
    Vector4d pos_in_map = t_map_world.inverse() * pos_in_world;

    int col = std::min(std::max(int(pos_in_map(0)* inv_resolution),0),int(num_of_colums - 1));
    int row = std::min(std::max(int(pos_in_map(1)* inv_resolution),0),int(num_of_rows - 1));
    Vector2i res; res<<col, row;
    return res;
}


 Vector2d bcd_planner::map2world(int col, int row){
    double x_in_map = col * resolution;
    double y_in_map = row * resolution;
    Vector4d pos_in_map; pos_in_map<<x_in_map, y_in_map, 0,1.0;
    Vector4d pos_in_world = t_map_world*pos_in_map;

    Vector2d res; res<<pos_in_world(0),pos_in_world(1);
    return res;
 }

 void bcd_planner::scal_polygon(std::vector<Vector2d>& wall_polygon,double robot_radius){
     std::vector<Vector2d> scal_polygons;
     int N = wall_polygon.size();
     int prev_idx,next_idx;
     for(int i = 0;i<N;i++){
        if(i == 0){
            prev_idx = N-1;
        }
        else{
            prev_idx = i -1;
        }
        if(i == (N-1)){
            next_idx = 0;
        }else{
            next_idx = i+1;
        }

         double x1 = wall_polygon[i](0) - wall_polygon[prev_idx](0);
         double y1 = wall_polygon[i](1) - wall_polygon[prev_idx](1);
         double x2 = wall_polygon[next_idx](0) - wall_polygon[i](0);
         double y2 = wall_polygon[next_idx](1) - wall_polygon[i](1);

         double PA = sqrt(x1*x1 + y1*y1);
         double PB = sqrt(x2*x2 + y2*y2);
         double vec_cross = x1*y2 - x2*y1;
         double sin_theta = vec_cross / (PA* PB);

         double dv = robot_radius / sin_theta;
         double v1_x = (dv/PA) * x1;
         double v1_y = (dv/PA) * y1;
         double v2_x = (dv/PB) * x2;
         double v2_y = (dv/PB) * y2;

         double PQ_x = v1_x - v2_x;
         double PQ_y = v1_y - v2_y;

         double Q_x = wall_polygon[i % N](0) + PQ_x;
         double Q_y = wall_polygon[i % N](1) + PQ_y;
         scal_polygons.push_back(Vector2d(Q_x, Q_y));
     }
     wall_polygon.assign(scal_polygons.begin(),scal_polygons.end());
 }

//  void bcd_planner::scal_polygon(std::vector<Vector3d>& wall_polygon,double robot_radius,std::vector<Vector2d>& scal_polygon){
//      int N = wall_polygon.size();
//      int prev_idx,next_idx;
//      for(int i = 0;i<N;i++){
//         if(i == 0){
//             prev_idx = N-1;
//         }
//         else{
//             prev_idx = i -1;
//         }
//         if(i == (N-1)){
//             next_idx = 0;
//         }else{
//             next_idx = i+1;
//         }

//          double x1 = wall_polygon[i](0) - wall_polygon[prev_idx](0);
//          double y1 = wall_polygon[i](1) - wall_polygon[prev_idx](1);
//          double x2 = wall_polygon[next_idx](0) - wall_polygon[i](0);
//          double y2 = wall_polygon[next_idx](1) - wall_polygon[i](1);

//          double PA = sqrt(x1*x1 + y1*y1);
//          double PB = sqrt(x2*x2 + y2*y2);
//          double vec_cross = x1*y2 - x2*y1;
//          double sin_theta = vec_cross / (PA* PB);

//          double dv = robot_radius / sin_theta;
//          double v1_x = (dv/PA) * x1;
//          double v1_y = (dv/PA) * y1;
//          double v2_x = (dv/PB) * x2;
//          double v2_y = (dv/PB) * y2;

//          double PQ_x = v1_x - v2_x;
//          double PQ_y = v1_y - v2_y;

//          double Q_x = wall_polygon[i % N](0) + PQ_x;
//          double Q_y = wall_polygon[i % N](1) + PQ_y;
//          scal_polygon.push_back(Vector2d(Q_x, Q_y));
//      }
//  }