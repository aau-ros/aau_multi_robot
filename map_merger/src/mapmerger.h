#ifndef MAPMERGER_H
#define MAPMERGER_H

#include "ros/ros.h"
#include "stdio.h"
#include "updatemanager.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/tracking.hpp"

#include "mapstitch.h"
#include "adhoc_communication/SendOccupancyGrid.h"
#include "adhoc_communication/SendMmRobotPosition.h"
#include "adhoc_communication/MmRobotPosition.h"
#include "adhoc_communication/MmListOfPoints.h"
//#include "adhoc_communication/PointFromOtherRobot.h"
#include "adhoc_communication/MmMapUpdate.h"
#include "adhoc_communication/MmControl.h"
#include "adhoc_communication/SendMmControl.h"
#include "adhoc_communication/SendMmMapUpdate.h"
#include "adhoc_communication/GetNeighbors.h"

#include "updateentry.h"

#include "map_merger/TransformPoint.h"
#include "map_merger/LogMaps.h"

#include "updateentry.h"

#include "adhoc_communication/ExpFrontier.h"
#include "adhoc_communication/ExpFrontierElement.h"
#include "visualization_msgs/MarkerArray.h"


enum MapMergerLog{
    LOG_GLOBAL_MAP          = 0x01,
    LOG_LOCAL_MAP           = 0x02,
    LOG_LOCAL_MAP_PROGRESS  = 0x04,
    LOG_GLOBAL_MAP_PROGRESS = 0x08
};


class MapMerger
{
public:
    MapMerger();
    void start();
    void waitForLocalMetaData();
    void waitForRobotInformation();
    bool transformPointSRV(map_merger::TransformPoint::Request &req,
                           map_merger::TransformPoint::Response &res);
    bool log_output_srv(map_merger::LogMaps::Request &req,
                        map_merger::LogMaps::Response &res);
    bool getHasLocalMap();
private:
    //Private Methods
    //callback methods
    void callback_map(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void callback_map_other(const adhoc_communication::MmMapUpdateConstPtr &msg);
    void callback_control(const adhoc_communication::MmControlConstPtr &msg);
    void callback_map_meta_data_local(const nav_msgs::MapMetaData::ConstPtr &msg);
    void callback_robot_status(const nav_msgs::MapMetaData::ConstPtr &msg);
    void callback_global_pub(const ros::TimerEvent &e);
    void callback_send_map(const ros::TimerEvent &e);
    void callback_send_position(const ros::TimerEvent &e);
    void callback_recompute_transform(const ros::TimerEvent &e);
    void callback_got_position(const nav_msgs::Odometry::ConstPtr &msg);
    void callback_got_position_network(const adhoc_communication::MmRobotPosition::ConstPtr &msg);
    void callback_new_robot(const std_msgs::StringConstPtr &msg);
    void callback_ask_other_robots(const ros::TimerEvent &e);
    void callback_remove_robot(const std_msgs::StringConstPtr &msg);

    void callback_write_maps(const ros::TimerEvent &e);

    //int since_last_trans_try;
    void computeOffsets();
    void computeTransform(int mapDataIndex);
    bool recomputeTransform(int mapDataIndex);
    void makeEmptyMapData(string robot_name,int height,int width,float resolution);
    void mergeMaps(nav_msgs::OccupancyGrid *mapToMerge,int min_x = 0,int min_y = 0,int max_x = -1,int max_y = -1);
    void callback_got_robot_for_data(const std_msgs::StringConstPtr &msg);
    void sendControlMessage(std::vector<int>* updateNumbers,std::string dest);

    void processMap(nav_msgs::OccupancyGrid *map,int index_in_mapdata);
    void processLocalMap(nav_msgs::OccupancyGrid * toInsert,int index);
    void processPosition(geometry_msgs::PoseStamped * pose);

    void updateMap(nav_msgs::OccupancyGrid *mapToUpdate,int index_of_transform);
    void updateMapArea(int map_index,nav_msgs::OccupancyGrid *newData,bool clear = false);
    int findTransformIndex(int robot_index);
    int findRobotIndex(int transform_index);
    void sendMapOverNetwork(string destination,std::vector<int>* containedUpdates,int start_row = 0,int start_collum = 0,int end_row = -1,int end_collum = -1);
    void sendMetaData(float res = 0.05);
    void sendBackAskedMapData(string robotName, std::vector<int> missingUpdates );
    nav_msgs::OccupancyGrid* getMapPart(int map_index,int start_x,int start_y,int width,int height);
    nav_msgs::OccupancyGrid* matToMap(const cv::Mat mat, nav_msgs::OccupancyGrid *forInfo);
    bool createLogPath();

    cv::Mat mapToMat(const nav_msgs::OccupancyGrid *map);
    cv::Mat transformImage(Mat img1,Mat trans);
    updateManager * updateMan;
    //Methots accsesing opencv
    ros::NodeHandle * nodeHandle;
    //Private Lists
    std::vector<std::string>* robots;
    std::vector<std::string>* robots_in_transform;
    //std::vector<geometry_msgs::PoseStamped> _position_otherrobots;
    std::vector<cv::Mat>* transforms;
    std::vector<bool>* new_data_maps;
    std::vector<UpdateEntry>* formerUpdates;
    std::vector<ros::Publisher> *robots_position_publisher;
    ros::Publisher  list_of_positions_publisher;
    //std::vector<adhoc_communication::
    adhoc_communication::MmListOfPoints * positions;
    int map_width,map_height,size,
        seconds_meta_timer,seconds_publish_timer,
        seconds_send_timer,seconds_recompute_transform,
	seconds_send_position,
        laser_range,updateCount;
    std::vector<nav_msgs::OccupancyGrid*>* map_data;
    std::vector<UpdateEntry*>* update_list;
    nav_msgs::OccupancyGrid * global_map, * local_map, * local_map_old;
    //Private variables
    bool    debug,
            splitted,
            has_local_map,
            exchange_position,
            local_map_new_data,
            force_recompute_all,
            global_map_ready;
//            convergente;
    cv::Mat lastTrans;
    ros::Publisher pub;
    geometry_msgs::PoseStamped * cur_position;
    float g_start_x,g_start_y;
    double max_trans_robot,max_rotation_robot;
    int update_seq;
    std::string topic_over_network,
                local_map_frame_id,
                local_map_topic,
                local_map_metadata_topic,
                position_local_robot_topic,
                map_meta_topic,
                position_other_robots_topic,
                robot_prefix,
                robot_name,
                control_topic,
                log_path,                   /// log path argument specified by launch file
                full_log_path,
                robot_hostname;              /// the full log path where to store files
    ros::Time time_start;
    double first_trans_x,first_trans_y;
    float old_middle_distance;
    ros::Timer send_position,recompute_transform_timer,global_timer_pub,send_map,ask_other_timer;

    std::vector<ros::Publisher> * pos_pub_other;
    ros::Publisher my_pos_pub;
    std::vector<visualization_msgs::MarkerArray> *  pos_array_other;
    visualization_msgs::MarkerArray pos_array_my;
    std::vector<int> * pos_seq_other;
    int pos_seq_my;
    bool turning;
};

#endif // MAPMERGER_Hd
