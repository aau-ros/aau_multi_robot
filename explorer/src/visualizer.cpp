void ExplorationPlanner::visualize_Frontiers()
{
        visualization_msgs::MarkerArray markerArray;

        for (int i = 0; i < frontiers.size(); i++)
        {
            visualization_msgs::Marker marker;

            marker.header.frame_id = move_base_frame;
            marker.header.stamp = ros::Time::now();
            marker.header.seq = frontier_seq_number++;
            marker.ns = "my_namespace";
            marker.id = frontier_seq_number;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.lifetime = ros::Duration(2); //TODO //F
            marker.pose.position.x = frontiers.at(i).x_coordinate;
            marker.pose.position.y = frontiers.at(i).y_coordinate;
            marker.pose.position.z = 0;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.2;
            marker.scale.y = 0.2;
            marker.scale.z = 0.2;

            marker.color.a = 1.0;

            if(frontiers.at(i).detected_by_robot == robot_name)
            {
                marker.color.r = 1.0;
                marker.color.g = 0.0;
                marker.color.b = 0.0;
            }
            else if(frontiers.at(i).detected_by_robot == 1)
            {
                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;
            }
            else if(frontiers.at(i).detected_by_robot == 2)
            {
                marker.color.r = 0.0;
                marker.color.g = 0.0;
                marker.color.b = 1.0;
            }
            else
            {
                marker.color.r = 0.0;
                marker.color.g = 0.0;
                marker.color.b = 1.0;
            }

            markerArray.markers.push_back(marker);
        }
        
        for(int j=0; j < ds_list.size(); j++) {
            visualization_msgs::Marker marker;

            marker.header.frame_id = move_base_frame;
            marker.header.stamp = ros::Time::now();
            marker.header.seq = frontier_seq_number++;
            marker.ns = "my_namespace";
            marker.id = frontier_seq_number;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.lifetime = ros::Duration(2); //TODO //F
            marker.pose.position.x = ds_list.at(j).x;
            marker.pose.position.y = ds_list.at(j).y;
            marker.pose.position.z = 0;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 1;
            marker.scale.y = 1;
            marker.scale.z = 0.1;

            marker.color.a = 1.0;

            marker.color.r = 0.5;
            marker.color.g = 0.0;
            marker.color.b = 0.0;

            markerArray.markers.push_back(marker);
        }
        
        
        

        pub_frontiers_points.publish <visualization_msgs::MarkerArray>(markerArray);
}

void ExplorationPlanner::visualize_visited_Frontiers()
{
        visualization_msgs::MarkerArray markerArray;

        for (int i = 0; i < visited_frontiers.size(); i++)
        {
            visualization_msgs::Marker marker;

            marker.header.frame_id = move_base_frame;
            marker.header.stamp = ros::Time::now();
            marker.header.seq = i+1;
            marker.ns = "my_namespace";
            marker.id = i+1;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = visited_frontiers.at(i).x_coordinate;
            marker.pose.position.y = visited_frontiers.at(i).y_coordinate;
            marker.pose.position.z = 0;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.2;
            marker.scale.y = 0.2;
            marker.scale.z = 0.2;

            marker.color.a = 1.0;

            if(visited_frontiers.at(i).detected_by_robot == robot_name)
            {
                marker.color.a = 0.2;
                marker.color.r = 1.0;
                marker.color.g = 0.0;
                marker.color.b = 0.0;
            }
            if(visited_frontiers.at(i).detected_by_robot == 1)
            {
                marker.color.a = 0.2;
                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;
            }
            if(visited_frontiers.at(i).detected_by_robot == 2)
            {
                marker.color.a = 0.2;
                marker.color.r = 0.0;
                marker.color.g = 0.0;
                marker.color.b = 1.0;
            }
            if(visited_frontiers.at(i).detected_by_robot == 3)
            {
                marker.color.a = 0.2;
                marker.color.r = 0.5;
                marker.color.g = 0.0;
                marker.color.b = 1.0;
            }
            if(visited_frontiers.at(i).detected_by_robot == 4)
            {
                marker.color.a = 0.2;
                marker.color.r = 0.0;
                marker.color.g = 0.5;
                marker.color.b = 1.0;
            }
            if(visited_frontiers.at(i).detected_by_robot == 5)
            {
                marker.color.a = 0.2;
                marker.color.r = 0.0;
                marker.color.g = 0.0;
                marker.color.b = 0.5;
            }
            if(visited_frontiers.at(i).detected_by_robot == 6)
            {
                marker.color.a = 0.2;
                marker.color.r = 0.5;
                marker.color.g = 0.0;
                marker.color.b = 0.5;
            }
            if(visited_frontiers.at(i).detected_by_robot == 7)
            {
                marker.color.a = 0.2;
                marker.color.r = 0.0;
                marker.color.g = 0.5;
                marker.color.b = 0.5;
            }
            if(visited_frontiers.at(i).detected_by_robot == 8)
            {
                marker.color.a = 0.2;
                marker.color.r = 0.0;
                marker.color.g = 0.5;
                marker.color.b = 0.0;
            }
            if(visited_frontiers.at(i).detected_by_robot == 9)
            {
                marker.color.a = 0.2;
                marker.color.r = 0.5;
                marker.color.g = 0.5;
                marker.color.b = 0.0;
            }
            if(visited_frontiers.at(i).detected_by_robot == 10)
            {
                marker.color.a = 0.2;
                marker.color.r = 0.5;
                marker.color.g = 0.5;
                marker.color.b = 0.5;
            }
            markerArray.markers.push_back(marker);
        }

        pub_visited_frontiers_points.publish <visualization_msgs::MarkerArray>(markerArray);
}
