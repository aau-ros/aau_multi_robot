//            // Visualize in RViz
//            visualization_msgs::Marker marker;

//            marker.header.frame_id = robot_prefix + "/map";
//            marker.header.stamp = ros::Time::now();
//            marker.header.seq = it->id;
//            marker.ns = "ds_position";
//            marker.id = it->id;
//            marker.type = visualization_msgs::Marker::SPHERE;
//            marker.action = visualization_msgs::Marker::ADD;
//            //marker.lifetime = ros::Duration(10); //TODO //F
//            marker.scale.x = 0.5;
//            marker.scale.y = 0.5;
//            marker.scale.z = 0.5;
//            marker.pose.position.x = it->x;
//            marker.pose.position.y = it->y;
//            marker.pose.position.z = 0;
//            marker.pose.orientation.x = 0.0;
//            marker.pose.orientation.y = 0.0;
//            marker.pose.orientation.z = 0.0;
//            marker.pose.orientation.w = 1.0;
//            marker.color.a = 1.0;
//            marker.color.r = 0.0;
//            marker.color.g = 0.0;
//            marker.color.b = 1.0;
//            pub_ds_position.publish< visualization_msgs::Marker >(marker);
//            pub_ds_position.publish< visualization_msgs::Marker >(marker);
//            pub_ds_position.publish< visualization_msgs::Marker >(marker);
//            pub_ds_position.publish< visualization_msgs::Marker >(marker);
