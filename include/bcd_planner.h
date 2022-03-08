/*
 * @Description: Boustrophedon Cellular Decomposition Planner
 */

#ifndef _BCD_PLANNER_H_
#define _BCD_PLANNER_H_

#include <iostream>
#include <vector>
#include <deque>
#include <map>
#include <algorithm>
#include <numeric> 
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <Eigen/Core>

using namespace Eigen;

enum EventType
{
    IN,                           //内部障碍物的envet类型
    IN_TOP,
    IN_BOTTOM,
    OUT,
    OUT_TOP,
    OUT_BOTTOM,
    INNER_IN,                     //INNER MAYBE 对应于polygon里面有HOLE的情况
    INNER_IN_TOP,
    INNER_IN_BOTTOM,          
    INNER_OUT,
    INNER_OUT_TOP,
    INNER_OUT_BOTTOM,           

    IN_EX,                       // 相对于WALL的event类型
    IN_TOP_EX,
    IN_BOTTOM_EX,
    OUT_EX,
    OUT_TOP_EX,
    OUT_BOTTOM_EX,
    INNER_IN_EX,
    INNER_IN_TOP_EX,
    INNER_IN_BOTTOM_EX,
    INNER_OUT_EX,
    INNER_OUT_TOP_EX,
    INNER_OUT_BOTTOM_EX,      

    MIDDLE,
    CEILING,
    FLOOR,
    UNALLOCATED
};

enum VisualizationMode{PATH_MODE, ROBOT_MODE};

const int TOPLEFT = 0;
const int BOTTOMLEFT = 1;
const int BOTTOMRIGHT = 2;
const int TOPRIGHT = 3;

const int palette_colors = 1530;

class Point2D
{
public:
    Point2D()
    {
        x = INT_MAX;
        y = INT_MAX;
    }
    Point2D(int x_pos, int y_pos)
    {
        x = x_pos;
        y = y_pos;
    }
    Point2D(const Point2D& point)
    {
        x = point.x;
        y = point.y;
    }
    int x;
    int y;
};

typedef std::vector<Point2D> Polygon;
typedef std::vector<Polygon> PolygonList;
typedef std::deque<Point2D> Edge;

class Event
{
public:
    Event(int obstacle_idx, int x_pos, int y_pos, EventType type=UNALLOCATED)
    {
        obstacle_index = obstacle_idx;
        x = x_pos;
        y = y_pos;
        event_type = type;
        original_index_in_slice = INT_MAX;
        isUsed = false;
    }

    int x;
    int y;
    int original_index_in_slice;
    int obstacle_index;
    EventType event_type;

    bool isUsed;
};

class CellNode
{
public:
    CellNode()
    {
        isVisited = false;
        isCleaned = false;
        parentIndex = INT_MAX;
        cellIndex = INT_MAX;
    }
    bool isVisited;
    bool isCleaned;
    Edge ceiling;
    Edge floor;

    int parentIndex;
    std::deque<int> neighbor_indices;

    int cellIndex;
};

bool operator<(const Point2D& p1, const Point2D& p2)
{
    return (p1.x < p2.x || (p1.x == p2.x && p1.y < p2.y));
}

bool operator<(const Event& e1, const Event& e2)
{
    return (e1.x < e2.x || (e1.x == e2.x && e1.y < e2.y) || (e1.x == e2.x && e1.y == e2.y && e1.obstacle_index < e2.obstacle_index));
}

bool operator==(const Point2D& p1, const Point2D& p2)
{
    return (p1.x==p2.x && p1.y==p2.y);
}

bool operator!=(const Point2D& p1, const Point2D& p2)
{
    return !(p1==p2);
}

class NavigationMessage
{
public:
    NavigationMessage()
    {
        foward_distance = 0.0;
        global_yaw_angle = 0.0;
        local_yaw_angle = 0.0;
    }
    void SetDistance(double dist)
    {
        foward_distance = dist;
    }
    void SetGlobalYaw(double global_yaw)
    {
        global_yaw_angle = global_yaw;
    }
    void SetLocalYaw(double local_yaw)
    {
        local_yaw_angle = local_yaw;
    }

    double GetDistance()
    {
        return foward_distance;
    }

    double GetGlobalYaw()
    {
        return global_yaw_angle;
    }

    double GetLocalYaw()
    {
        return local_yaw_angle;
    }

    void GetMotion(double& dist, double& global_yaw, double& local_yaw)
    {
        dist = foward_distance;
        global_yaw = global_yaw_angle;
        local_yaw = local_yaw_angle;
    }
    void Reset()
    {
        foward_distance = 0.0;
        global_yaw_angle = 0.0;
        local_yaw_angle = 0.0;
    }

private:
    double foward_distance;
    // 欧拉角表示，逆时针为正，顺时针为负
    double global_yaw_angle;
    double local_yaw_angle;
};


class bcd_planner
{
    public:
       bcd_planner(){}

       bcd_planner(std::vector<std::vector<cv::Point>>& wall_polygon, std::vector<std::vector<cv::Point>>&obstcal_polygon, double robot_radius,std::deque<Point2D>& path);
       bcd_planner(std::vector<std::vector<Vector2d>>& wall_polygon, std::vector<std::vector<Vector2d>>&obstcal_polygon, double robot_radius,std::deque<Vector2d>& path);
       bcd_planner(std::vector<std::vector<Vector3d>>& wall_polygon, std::vector<std::vector<Vector3d>>&obstcal_polygon, double robot_radius);

       void TestAllExamples();

        

    private:

        //data
        double robot_radius_;

        void plan(std::vector<std::vector<cv::Point>>& wall_polygon, std::vector<std::vector<cv::Point>>&obstcal_polygon, double robot_radius, std::deque<Point2D>& path);

        int WrappedIndex(int index, int list_length);
        void WalkThroughGraph(std::vector<CellNode>& cell_graph, int cell_index, int& unvisited_counter, std::deque<CellNode>& path); //DFS
        std::deque<CellNode> GetVisittingPath(std::vector<CellNode>& cell_graph, int first_cell_index);
        std::vector<Point2D> ComputeCellCornerPoints(const CellNode& cell);

        std::vector<int> DetermineCellIndex(std::vector<CellNode>& cell_graph, const Point2D& point);
        std::deque<Point2D> GetBoustrophedonPath(std::vector<CellNode>& cell_graph, CellNode cell, int corner_indicator, int robot_radius);
       
        std::vector<Event> InitializeEventList(const Polygon& polygon, int polygon_index);
        void AllocateObstacleEventType(const cv::Mat& map, std::vector<Event>& event_list);
        void AllocateWallEventType(const cv::Mat& map, std::vector<Event>& event_list);
        std::vector<Event> GenerateObstacleEventList(const cv::Mat& map, const PolygonList& polygons);
        std::vector<Event> GenerateWallEventList(const cv::Mat& map, const Polygon& external_contour);
        std::deque<std::deque<Event>> SliceListGenerator(const std::vector<Event>& wall_event_list, const std::vector<Event>& obstacle_event_list);
      
        void ExecuteOpenOperation(std::vector<CellNode>& cell_graph, int curr_cell_idx, Point2D in, Point2D c, Point2D f, bool rewrite = false);
        void ExecuteCloseOperation(std::vector<CellNode>& cell_graph, int top_cell_idx, int bottom_cell_idx, Point2D c, Point2D f, bool rewrite = false);
        void ExecuteCeilOperation(std::vector<CellNode>& cell_graph, int curr_cell_idx, const Point2D& ceil_point);
        void ExecuteFloorOperation(std::vector<CellNode>& cell_graph, int curr_cell_idx, const Point2D& floor_point);
        void ExecuteOpenOperation(std::vector<CellNode>& cell_graph, int curr_cell_idx, Point2D in_top, Point2D in_bottom, Point2D c, Point2D f, bool rewrite = false);
        void ExecuteInnerOpenOperation(std::vector<CellNode>& cell_graph, Point2D inner_in);
        void ExecuteInnerOpenOperation(std::vector<CellNode>& cell_graph, Point2D inner_in_top, Point2D inner_in_bottom);
        void ExecuteInnerCloseOperation(std::vector<CellNode>& cell_graph, int curr_cell_idx, Point2D inner_out);
        void ExecuteInnerCloseOperation(std::vector<CellNode>& cell_graph, int curr_cell_idx, Point2D inner_out_top, Point2D inner_out_bottom);

        void DrawCells(cv::Mat& map, const CellNode& cell, cv::Scalar color=cv::Scalar(100, 100, 100));

        int CountCells(const std::deque<Event>& slice, int curr_idx);
        std::deque<Event> FilterSlice(const std::deque<Event>& slice);
        void ExecuteCellDecomposition(std::vector<CellNode>& cell_graph, std::vector<int>& cell_index_slice, std::vector<int>& original_cell_index_slice, const std::deque<std::deque<Event>>& slice_list);
        Point2D FindNextEntrance(const Point2D& curr_point, const CellNode& next_cell, int& corner_indicator);

        std::deque<Point2D> WalkInsideCell(CellNode cell, const Point2D& start, const Point2D& end);
        std::deque<std::deque<Point2D>> FindLinkingPath(const Point2D& curr_exit, Point2D& next_entrance, int& corner_indicator, CellNode curr_cell, const CellNode& next_cell);
        std::deque<Point2D> WalkCrossCells(std::vector<CellNode>& cell_graph, std::deque<int> cell_path, const Point2D& start, const Point2D& end, int robot_radius);

        std::deque<int> FindShortestPath(std::vector<CellNode>& cell_graph, const Point2D& start, const Point2D& end);
        void InitializeColorMap(std::deque<cv::Scalar>& JetColorMap, int repeat_times);
        void UpdateColorMap(std::deque<cv::Scalar>& JetColorMap);

        int ComputeRobotRadius(const double& meters_per_pix, const double& robot_size_in_meters);
        cv::Mat1b ReadMap(const std::string& map_file_path);
        cv::Mat1b PreprocessMap(const cv::Mat1b& original_map);
        void ExtractRawContours(const cv::Mat& original_map, std::vector<std::vector<cv::Point>>& raw_wall_contours, std::vector<std::vector<cv::Point>>& raw_obstacle_contours);
        void ExtractContours(const cv::Mat& original_map, std::vector<std::vector<cv::Point>>& wall_contours, std::vector<std::vector<cv::Point>>& obstacle_contours, int robot_radius=0);

        PolygonList ConstructObstacles(const cv::Mat& original_map, const std::vector<std::vector<cv::Point>>& obstacle_contours);
        Polygon ConstructDefaultWall(const cv::Mat& original_map);
        Polygon ConstructWall(const cv::Mat& original_map, std::vector<cv::Point>& wall_contour);
        std::vector<CellNode> ConstructCellGraph(const cv::Mat& original_map, const std::vector<std::vector<cv::Point>>& wall_contours, const std::vector<std::vector<cv::Point>>& obstacle_contours, const Polygon& wall, const PolygonList& obstacles);
        
        std::deque<std::deque<Point2D>> StaticPathPlanning(const cv::Mat& map, std::vector<CellNode>& cell_graph, const Point2D& start_point, int robot_radius, bool visualize_cells, bool visualize_path, int color_repeats=10);
        std::deque<Point2D> ReturningPathPlanning(cv::Mat& map, std::vector<CellNode>& cell_graph, const Point2D& curr_pos, const Point2D& original_pos, int robot_radius, bool visualize_path);
        std::deque<Point2D> FilterTrajectory(const std::deque<std::deque<Point2D>>& raw_trajectory);
        void VisualizeTrajectory(const cv::Mat& original_map, const std::deque<Point2D>& path, int robot_radius, int vis_mode, int time_interval=10, int colors=palette_colors);

          
        double ComputeYaw(Eigen::Vector2d curr_direction, Eigen::Vector2d base_direction);
        double ComputeDistance(const Point2D& start, const Point2D& end, double meters_per_pix);
        std::vector<NavigationMessage> GetNavigationMessage(const Eigen::Vector2d& curr_direction, std::deque<Point2D> pos_path, double meters_per_pix);
    
       const int UP = 0, UPRIGHT = 1, RIGHT = 2, DOWNRIGHT = 3, DOWN = 4, DOWNLEFT = 5, LEFT = 6, UPLEFT = 7, CENTER = 8;
       const std::vector<int> map_directions = {UP, UPRIGHT, RIGHT, DOWNRIGHT, DOWN, DOWNLEFT, LEFT, UPLEFT};
       int GetFrontDirection(const Point2D& curr_pos, const Point2D& next_pos);
       int GetBackDirection(int front_direction);
       int GetLeftDirection(int front_direction);
       int GetRightDirection(int front_direction);
       std::vector<int> GetFrontDirectionCandidates(int front_direction);
       std::vector<int> GetBackDirectionCandidates(int front_direction);
       std::vector<int> GetLeftDirectionCandidates(int front_direction);
       std::vector<int> GetRightDirectionCandidates(int front_direction);

       Point2D GetNextPosition(const Point2D& curr_pos,  int direction, int steps);
       bool CollisionOccurs(const cv::Mat& map, const Point2D& curr_pos, int detect_direction, int robot_radius);
       bool WalkAlongObstacle(const cv::Mat& map,      const Point2D& obstacle_origin,               const Point2D& contouring_origin,
                       int detecting_direction, const std::vector<int>& direction_candidates,
                       Point2D& curr_pos,       int first_turning_direction,                  int second_turning_direction,
                       Polygon& obstacle,       Polygon& new_obstacle,                        bool& isObstacleCompleted,
                       std::deque<Point2D>& contouring_path,                                  int robot_radius);
      
      Polygon GetNewObstacle(const cv::Mat& map, Point2D origin, int front_direction, std::deque<Point2D>& contouring_path, int robot_radius);
      int GetCleaningDirection(const CellNode& cell, Point2D exit);
      std::deque<std::deque<Point2D>> LocalReplanning(cv::Mat& map, CellNode outer_cell, const PolygonList& obstacles, const Point2D& curr_pos, std::vector<CellNode>& curr_cell_graph, int cleaning_direction, int robot_radius, bool visualize_cells=false, bool visualize_path=false);

      std::deque<Point2D> DynamicPathPlanning(cv::Mat& map, const std::vector<CellNode>& global_cell_graph, std::deque<std::deque<Point2D>> global_path, int robot_radius, bool returning_home, bool visualize_path, int color_repeats=10);
      void MoveAsPathPlannedTest(cv::Mat& map, double meters_per_pix, const Point2D& start, const std::vector<NavigationMessage>& motion_commands);

      //for test
      std::vector<std::vector<cv::Point>> ConstructHandcraftedContours1();
      std::vector<std::vector<cv::Point>> ConstructHandcraftedContours2();
      std::vector<std::vector<cv::Point>> ConstructHandcraftedContours3();
      std::vector<std::vector<cv::Point>> ConstructHandcraftedContours4();
      std::vector<std::vector<cv::Point>> ConstructHandcraftedContours5();
      std::vector<std::vector<cv::Point>> ConstructHandcraftedContours6();

      void CheckObstaclePointType(cv::Mat& map, const Polygon& obstacle);
      void CheckWallPointType(cv::Mat& map, const Polygon& wall);
      void CheckPointType(const cv::Mat& map, const Polygon& wall, const PolygonList& obstacles);
      void CheckSlicelist(const std::deque<std::deque<Event>>& slice_list);
      void CheckExtractedContours(const cv::Mat& map, const std::vector<std::vector<cv::Point>>& contours);
      void CheckGeneratedCells(const cv::Mat& map, const std::vector<CellNode>& cell_graph);
      void CheckPathNodes(const std::deque<std::deque<Point2D>>& path);
      void CheckPathConsistency(const std::deque<Point2D>& path);
      void CheckMotionCommands(const std::vector<NavigationMessage>& navigation_messages);

      void StaticPathPlanningExample1();
      void StaticPathPlanningExample2();
      void StaticPathPlanningExample3();
      void StaticPathPlanningExample4();
      void StaticPathPlanningExample5();
      void StaticPathPlanningExample6();
      

      
};



















#endif