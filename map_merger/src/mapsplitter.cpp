#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include "numeric"
using namespace std;

void callback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
nav_msgs::OccupancyGrid* getMapPart(nav_msgs::OccupancyGrid *tmp, int start_x, int start_y, int width, int height);
int main(int argc, char **argv)
{
    ROS_INFO("Start map_splitter");
    ros::init(argc,argv,"map_splitter");
    string topic;
    ros::NodeHandle n;
    n.param<string>("/map_splitter/map_topic",topic,"map");
    ROS_INFO("Subscribed Topic: %s",topic.c_str());

    ros::Subscriber  sub = n.subscribe(topic,1000,&callback);
    ROS_DEBUG("Init Subscriber");
    ROS_INFO("Splitter Waits");
    ros::Duration(6).sleep();
    ros::spin();
}

void callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    int size;
    ros::NodeHandle n;
    string frame,pub_frame,pub_topic_map,pub_topic_meta;

    n.param<int>("/map_splitter/size",size,32);
    n.param<string>("/map_splitter/map_frame",frame,"map");
    n.param<string>("/map_splitter/pub_topic_map",pub_topic_map,"map");
    n.param<string>("/map_splitter/pub_topic_meta",pub_topic_meta,"map_meta");
    //ROS_INFO("Wanted mapframe: %s",frame.c_str());
    n.param<string>("/map_splitter/pub_frame",pub_frame,"map");
    //ROS_INFO("Published frame: %s",pub_frame.c_str());
    ros::Publisher pub_map = n.advertise<nav_msgs::OccupancyGrid>(pub_topic_map,3);
    ros::Publisher pub_meta = n.advertise<nav_msgs::OccupancyGrid>(pub_topic_meta,3);
    //get the message
    nav_msgs::OccupancyGrid   local_map = *msg.get();
    //check if it is the map i want
    string frame_id = local_map.header.frame_id;
    if(local_map.header.frame_id != frame)
    {
        if(frame_id != pub_frame)
            ROS_WARN("Got Map with wrong frame_id[%s]",frame_id.c_str());
        return;
    }
    //prepare it for the publish
    ROS_INFO("Got Map to split");
    ROS_INFO("Publish meta_data_packet");
    nav_msgs::OccupancyGrid * meta_data = new nav_msgs::OccupancyGrid();
    meta_data->header.frame_id = pub_frame;
    meta_data->info.height = local_map.info.height;
    meta_data->info.width = local_map.info.width;
    meta_data->info.resolution = local_map.info.resolution;

    ros::Duration(3).sleep();
    for(int row = 0; row < local_map.info.height-size;row+=size)
    {
        for(int collum = 0; collum < local_map.info.width-size;collum+=size)
        {
            //ROS_INFO("row:%icollum:%i:",row,collum);
            nav_msgs::OccupancyGrid * t = getMapPart(&local_map,row,collum,size,size);
            int sum_elements = std::accumulate(t->data.begin(),t->data.end(),0);
            if(sum_elements == (-1)* t->data.size())
            {
                ROS_DEBUG("Map is empty, do not publish");
                continue;
            }
            ROS_INFO("Published Mapframe:%s||r:%i;c:%i",pub_frame.c_str(),row,collum);
            t->header.frame_id = pub_frame;
            pub_map.publish(*t);
            pub_meta.publish(*meta_data);

            delete t;
            ros::Duration(0.2).sleep();
        }

    }
}

nav_msgs::OccupancyGrid* getMapPart(nav_msgs::OccupancyGrid *tmp, int start_x, int start_y, int width, int height)
{
    nav_msgs::OccupancyGrid* part = new nav_msgs::OccupancyGrid();
    for(int row = 0; row < height;row ++)
    {
        for(int collum = 0; collum < width;collum++)
        {
           part->data.push_back(tmp->data.at((row+start_x)*tmp->info.width+(collum + start_y)));
        }
    }
    part->header = tmp->header;
    part->info.origin.position.x = start_x;
    part->info.origin.position.y = start_y;
    part->info.height = height;
    part->info.width = width;
    return part;
}
