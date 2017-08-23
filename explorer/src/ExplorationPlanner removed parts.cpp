// A utility function to find the vertex with minimum distance value, from
// the set of vertices not yet included in shortest path tree
int minDistance(int dist[], unsigned long size, bool sptSet[])
{
   // Initialize min value
   int min = INT_MAX, min_index = -1;
  
   for (int v = 0; v < size; v++)
     if (sptSet[v] == false && dist[v] <= min)
         min = dist[v], min_index = v;
  
   return min_index;
}
  
// Funtion that implements Dijkstra's shortest path algorithm
// from a given source node to a given destination node
// for a graph represented using adjacency matrix representation
float dijkstra(std::vector <std::vector<float> > graph, int src, int dest)
{   
    unsigned long V = graph.size();
     int dist[V];     // The output array.  dist[i] will hold the shortest
                      // distance from src to i
  
     bool sptSet[V]; // sptSet[i] will true if vertex i is included in shortest
                     // path tree or shortest distance from src to i is finalized
  
     // Initialize all distances as INFINITE and stpSet[] as false
     for (int i = 0; i < V; i++)
        dist[i] = INT_MAX, sptSet[i] = false;
  
     // Distance of source vertex from itself is always 0
     dist[src] = 0;
  
     // Find shortest path for all vertices
     for (int count = 0; count < V-1; count++)
     {
       // Pick the minimum distance vertex from the set of vertices not
       // yet processed. u is always equal to src in first iteration.
       int u = minDistance(dist, V, sptSet);
       
       if(u < 0)
         ROS_ERROR("Unconnected components!!!");
  
       // Mark the picked vertex as processed
       sptSet[u] = true;
  
       // Update dist value of the adjacent vertices of the picked vertex.
       for (int v = 0; v < V; v++) {
  
         // Update dist[v] only if is not in sptSet, there is an edge from 
         // u to v, and total weight of path from src to  v through u is 
         // smaller than current value of dist[v]
         if (!sptSet[v] && graph[u][v] && dist[u] != INT_MAX 
                                       && dist[u]+graph[u][v] < dist[v])
            dist[v] = dist[u] + graph[u][v];
            
         if(u == dest)
            break;
       
       }
     }

     return (float) dist[dest];
}

bool ExplorationPlanner::selectClusterBasedOnAuction(std::vector<double> *goal, std::vector<int> *cluster_in_use_already_count, std::vector<std::string> *robot_str_name_to_return)
{
    ROS_INFO("Select the best cluster based on auction bids");

    int own_row_to_select_cluster = 0;
    int own_column_to_select = 0;

    /*
     * Select the method to select clusters from the matrix.
     * 0 ... select nearest cluster from OWN calculations
     * 1 ... gather information of others, estimate trajectory length based on
     *       distance information and select the optimal cluster using the
     */
    int method_used = 2;

    int count = 0;
    int auction_cluster_element_id = -1;

    auction_element_t auction_elements;
    auction_pair_t auction_pair;

    /*
     * Calculate my own auction BIDs to all my clusters
     * and store them in the auction vector
     */
    ROS_INFO("Cluster Size: %u", clusters.size());
    for(int i = 0; i < clusters.size(); i++)
    {
        ROS_INFO("Calculate cluster with ID: %d", clusters.at(i).id);
        int my_auction_bid = calculateAuctionBID(clusters.at(i).id, trajectory_strategy);
        if(my_auction_bid == -1)
        {
            ROS_WARN("Own BID calculation failed");
        }
        ROS_INFO("Own bid calculated for cluster '%d' is '%d'", clusters.at(i).id, my_auction_bid);
        auction_pair.cluster_id = clusters.at(i).id;
        auction_pair.bid_value = my_auction_bid;
        auction_elements.auction_element.push_back(auction_pair);
    }
    auction_elements.robot_id = robot_name;
    auction.push_back(auction_elements);


    /*
     * Remove all clusters which have previously been selected by other robots
     */

    std::vector<int> clusters_to_erase;
    for(int j = 0; j < clusters.size(); j++)
    {
        for(int i = 0; i < already_used_ids.size(); i++)
        {
            if(already_used_ids.at(i) == clusters.at(j).id)
            {
                /*Already used cluster found, therefore erase it*/
                for(int m = 0; m < auction.size(); m++)
                {
                    for(int n = 0; n < auction.at(m).auction_element.size(); n++)
                    {
                        if(auction.at(m).auction_element.at(n).cluster_id == already_used_ids.at(i))
                        {
                            ROS_INFO("Already used Cluster %d found .... unconsider it",auction.at(m).auction_element.at(n).cluster_id);
//                            auction.at(m).auction_element.erase(auction.at(m).auction_element.begin()+n);
//                            n--;
                              auction.at(m).auction_element.at(n).bid_value = 10000;
                        }
                    }
                }
            }
        }
    }


    ROS_INFO("Matrix");

    /*
     * Calculate the matrix size
     */
    int col = 0, row = 0;

    int robots_operating;
    nh.param<int>("robots_in_simulation",robots_operating, 1);
    ROS_INFO("robots operating: %d", robots_operating);

    row = auction.size();


    /* FIXME */
//    if(auction.size() < robots_operating)
//    {
//        row = robots_operating;
//    }else
//    {
//        row = auction.size();
//    }


//    for(int i= 0; i < auction.size(); i++)
//    {
//        if(auction.at(i).auction_element.size() > col)
//        {
//            col = auction.at(i).auction_element.size();
//        }
//    }
    col = clusters.size();
    ROS_INFO("robots: %d     clusters: %d", row, col);
    /*
    * create a matrix with boost library
    */

    int max_size = std::max(col,row);
    boost::numeric::ublas::matrix<double> m(max_size,max_size);

    ROS_INFO("Empty the matrix");
    /*
     * initialize the matrix with all -1
     */
    for(int i = 0; i < m.size1(); i++)
    {
        for(int j = 0; j < m.size2(); j++)
        {
            m(i,j) = 0;
        }
    }


    ROS_INFO("Fill matrix");
    /*
     * Now fill the matrix with real auction values and use
     * a method for the traveling salesman problem
     *
     * i ... number of robots
     * j ... number of clusters
     */
//    ROS_INFO("robots: %d   clusters: %d", row, col);
    for(int i = 0; i < row; i++)
    {
        ROS_INFO("                 ");
        ROS_INFO("****** i: %d   auction size: %u ******", i, auction.size());
        for(int j = 0; j < col; j++)
        {
            ROS_INFO("j: %d   cluster size: %u", j, clusters.size());
            bool found_a_bid = false;
            bool cluster_valid_flag = false;

//            if(i < auction.size())
//            {
                for(int n = 0; n < auction.at(i).auction_element.size(); n++)
                {
                    cluster_valid_flag = true;
//                    if(j < clusters.size())
//                    {
//                        ROS_INFO("j: %d    smaller then clusters.size(): %u", j, clusters.size());
                        if(clusters.at(j).id == auction.at(i).auction_element.at(n).cluster_id && auction.at(i).auction_element.at(n).bid_value >= 0)
                        {
        //                    /*
        //                     * Check if duplicates exist, if so change them
        //                     */
        //                    bool duplicate_exist = false;
        //                    for(int k = 0; k < row; k++)
        //                    {
        //                        for(int l = 0; l < col; l++)
        //                        {
        //                            if(m(l,k) == auction.at(i).auction_element.at(n).bid_value)
        //                            {
        //                                ROS_INFO("Duplicate found");
        //                                duplicate_exist = true;
        //                            }
        //                        }
        //                    }
        //
        //                    if(duplicate_exist == false)
        //                    {
        //                        ROS_INFO("assign value");
                                m(j,i) = auction.at(i).auction_element.at(n).bid_value;
        //                        ROS_INFO("done");
        //                    }else
        //                    {
        //                        m(j,i) = auction.at(i).auction_element.at(n).bid_value + 5;
        //                    }
                            found_a_bid = true;
                            break;
                        }
//                    }else if(j >= clusters.size())
//                    {
//                        ROS_ERROR("j >= clusters.size()");
//                        cluster_valid_flag = false;
//                        row = clusters.size();
//                        break;
//                    }
                }
//            }else if(i >= auction.size())
//            {
//                ROS_ERROR("i >= auction.size()");
//                col = auction.size();
//                break;
//            }

            ROS_INFO("Cluster elements checked, found BID: %d", found_a_bid);

            /*
             * The auction does not contain a BID value for this cluster,
             * therefore try to estimate it, using the other robots position.
             */
            if(found_a_bid == false) // && cluster_valid_flag == true)
            {
                ROS_INFO("No BID received ...");
                int distance = -1;
                other_robots_position_x = -1;
                other_robots_position_y = -1;

                ROS_INFO("---- clusters: %u element size: %u  robots positions: %u ----", clusters.size(), clusters.at(j).cluster_element.size(), other_robots_positions.positions.size());
                for(int d = 0; d < clusters.at(j).cluster_element.size(); d++)
                {
                    ROS_INFO("Access clusters.at(%d).cluster_element.at(%d)", j, d);
                    position_mutex.lock();
                    /*Check position of the current robot (i)*/
                    for(int u = 0; u < other_robots_positions.positions.size(); u++)
                    {
                        adhoc_communication::MmPoint position_of_robot;
                        position_of_robot = other_robots_positions.positions.at(u);

                        if(robot_prefix_empty_param == true)
                        {
                            ROS_INFO("position of robot: %s   compare with auction robot: %s", position_of_robot.src_robot.c_str(), auction.at(i).detected_by_robot_str.c_str());
                            if(position_of_robot.src_robot.compare(auction.at(i).detected_by_robot_str) == 0)
                            {
                                other_robots_position_x = position_of_robot.x;
                                other_robots_position_y = position_of_robot.y;

                                break;
                            }
                            else
                            {
                                ROS_ERROR("Unable to look up robots position!");
                            }
                        }else
                        {
                            int robots_int_id = atoi(position_of_robot.src_robot.substr(6,1).c_str());
                            ROS_INFO("Robots int id: %d", robots_int_id);
                            if(auction.at(i).robot_id == robots_int_id)
                            {
                                other_robots_position_x = position_of_robot.x;
                                other_robots_position_y = position_of_robot.y;

                                ROS_INFO("Robots %d     position_x: %f     position_y: %f", robots_int_id, other_robots_position_x, other_robots_position_y);
                                break;
                            }else
                            {
//                                ROS_ERROR("robot id requested: %d      Current robots position id: %d", auction.at(i).robot_id, robots_int_id);
                                ROS_ERROR("Unable to look up robots position!");
                            }
                        }
                    }
                    position_mutex.unlock();

//                    ROS_INFO("Got robot position");
                    if(other_robots_position_x != -1 && other_robots_position_y != -1)
                    {

                        if(trajectory_strategy == "trajectory")
                        {
                            distance = trajectory_plan(other_robots_position_x, other_robots_position_y, clusters.at(j).cluster_element.at(d).x_coordinate, clusters.at(j).cluster_element.at(d).y_coordinate);
                            ROS_INFO("estimated TRAJECTORY distance is: %d", distance);

                        }else if(trajectory_strategy == "euclidean" && j < clusters.size() && d < clusters.at(j).cluster_element.size())
                        {
                            double x = clusters.at(j).cluster_element.at(d).x_coordinate - other_robots_position_x;
                            double y = clusters.at(j).cluster_element.at(d).y_coordinate - other_robots_position_y;
                            distance = x * x + y * y;
                            ROS_INFO("estimated EUCLIDEAN distance is: %d", distance);

                            if(distance != -1)
                                break;
                        }
                    }
                    /*
                     * Check if the distance calculation is plausible.
                     * The euclidean distance to this point need to be smaller then
                     * the simulated trajectory path. If this stattement is not valid
                     * the trajectory calculation has failed.
                     */
                    if(distance != -1)
                    {
                        double x = clusters.at(j).cluster_element.at(d).x_coordinate - other_robots_position_x;
                        double y = clusters.at(j).cluster_element.at(d).y_coordinate - other_robots_position_y;
                        double euclidean_distance = x * x + y * y;

//                        ROS_INFO("Euclidean distance: %f   trajectory_path: %f", sqrt(euclidean_distance), distance * costmap_ros_->getCostmap()->getResolution());
                        if (distance * costmap_ros_->getCostmap()->getResolution() <= sqrt(euclidean_distance)*0.95)
                        {
                            ROS_ERROR("Euclidean distance is smaller then the trajectory path at recalculation");
                            distance = -1;
                        }else
                        {
                            break;
                        }
                    }
                }
                if(distance == -1)
                {
                    /*
                     * Cluster is definitely unknown. Therefore assign a high value
                     * to ensure not to select this cluster
                     */

                    ROS_ERROR("Unable to calculate the others BID at all");
                    m(j,i) = 10000;
                }
                else
                {
                    ROS_INFO("Estimated trajectory length: %d", distance);

//                    /*
//                     * Check if duplicates exist, if so change them
//                     */
//                    bool duplicate_exist = false;
//                    for(int k = 0; k < row; k++)
//                    {
//                        for(int l = 0; l < col; l++)
//                        {
//                            if(m(l,k) == distance)
//                            {
//                                ROS_INFO("Duplicate found");
//                                duplicate_exist = true;
//                            }
//                        }
//                    }
//
//                    if(duplicate_exist == false)
//                    {
                        m(j,i) = distance;
//                    }else
//                    {
//                        m(j,i) = distance +5;
//                    }
                }
            }
            ROS_INFO("Column filled");
        }
        ROS_INFO("No columns left. Check next robot");
    }
//    std::cout << m << std::endl;

    ROS_INFO("Completed");

    /*
     * If the number of clusters is less the number of robots,
     * the assigning algorithmn would not come to a solution. Therefore
     * select the nearest cluster
     */
    if(col < row)
    {
        ROS_ERROR("Number of clusters is less the number of robots. Select the closest");
        if(col == 0)
        {
            ROS_ERROR("No cluster anymore available");
            /*No clusters are available at all*/
            return false;
        }
        method_used = 0;
    }

    /*
     * Select the strategy how to select clusters from the matrix
     *
     * 0 ... select nearest cluster
     * 1 ...
     */
    if(method_used == 0)
    {

        /*
         * Select the nearest cluster (lowest bid value)
         * THE ROBOTS OWN BIDS ARE IN THE LAST ROW
         * initialize the bid value as first column in the first row
         */
//        int smallest_bid_value = auction.back().auction_element.front().bid_value;

//        ROS_INFO("smallest_bid_value at initialization: %d", smallest_bid_value);
        ROS_INFO("Columns left: %d", col);
//        for(int j = 0; j < col; j++)
//        {
//            ROS_INFO("col: %d", j);
//            if(auction.back().auction_element.at(j).bid_value <= smallest_bid_value && auction.back().auction_element.at(j).bid_value != -1)
//            {
//                smallest_bid_value = auction.back().auction_element.at(j).bid_value;
//                auction_cluster_element_id = auction.back().auction_element.at(j).cluster_id;
//            }
//        }
        if(auction.size() > 0)
        {
            if(auction.back().auction_element.size() > 0)
            {
                auction_cluster_element_id = auction.back().auction_element.front().cluster_id;
            }else
            {
                 return false;
            }
        }
    }
    if(method_used == 1)
    {
       /*
        * Use the Ungarische method/ KuhnMunkresAlgorithm in order to figure out which
        * cluster the most promising one is for myself
        */

        /* an example cost matrix */
        int mat[col*row];
        int counter = 0;
        for(int j = 0; j < col; j++)
        {
            for(int i = 0; i < row; i++)
            {
                mat[counter] = m(j,i);
                counter++;
            }
        }

        std::vector< std::vector<int> > m2 = array_to_matrix(mat,col,row);

        /*
         * Last row in the matrix contains BIDs of the robot itself
         * Remember the max row count before filling up with zeros.
         */
        own_row_to_select_cluster = row-1;
        ROS_INFO("Own row: %d", own_row_to_select_cluster);

        /* an example cost matrix */
        int r[3*3] =  {14,15,15,30,1,95,22,14,12};
        std::vector< std::vector<int> > m3 = array_to_matrix(r,3,3);

       /* initialize the gungarian_problem using the cost matrix*/
       Hungarian hungarian(m2, col, row, HUNGARIAN_MODE_MINIMIZE_COST);

//        Hungarian hungarian(m3, 3, 3, HUNGARIAN_MODE_MINIMIZE_COST);

       fprintf(stderr, "cost-matrix:");
       hungarian.print_cost();

       /* solve the assignment problem */
       for(int i = 0; i < 5; i++)
       {
           if(hungarian.solve() == true)
               break;
       }

       const std::vector< std::vector<int> > assignment = hungarian.assignment();

       /* some output */
       fprintf(stderr, "assignment:");
       hungarian.print_assignment();

//       for(int i = 0; i < assignment.size(); i++)
//       {
//           ROS_ERROR("---- %d -----", i);
//           for(int j = 0; j < assignment.at(i).size(); j++)
//           {
//               ROS_INFO("%d", assignment.at(i).at(j));
//           }
//       }


       for(int i = 0; i < assignment.size(); i++)
       {
           if(assignment.at(i).at(own_row_to_select_cluster) == 1)
           {
               auction_cluster_element_id = clusters.at(i).id;
               ROS_INFO("Selected Cluster at position : %d   %d",i ,own_row_to_select_cluster);
               break;
           }
       }

    }
    if(method_used == 2)
    {

	Matrix<double> mat = convert_boost_matrix_to_munkres_matrix<double>(m);
        ROS_INFO("Matrix (%ux%u):",mat.rows(),mat.columns());

	// Display begin matrix state.
	for ( int new_row = 0 ; new_row < mat.rows(); new_row++ ) {
            if(new_row > 9)
            {
                int rows_left = mat.rows() - new_row + 1;
                std::cout << "... (" << rows_left << " more)";
                break;
            }
            for ( int new_col = 0 ; new_col < mat.columns(); new_col++ ) {
                if(new_col > 9)
                {
                    int columns_left = mat.columns() - new_col + 1;
                    std::cout << "... (" << columns_left << " more)";
                    break;
                }
                std::cout.width(2);
                std::cout << mat(new_row,new_col) << " ";
            }
            std::cout << std::endl;
	}
	std::cout << std::endl;



//        for(int i = 0; i < mat.columns(); i++)
//        {
//            for(int j = 0; j < mat.rows(); j++)
//            {
//                /*Inner Matrix*/
////                bool element_found = false;
//                for(int n = 0; n < mat.columns(); n++)
//                {
//                    for(int m = 0; m < mat.rows(); m++)
//                    {
//                        if(abs(mat(i,j) - mat(n,m)) <= 20 && abs(mat(i,j) - mat(n,m)) != 0 && (mat(i,j) != 0 && mat(n,m) != 0))
//                        {
//                            mat(i,j) = 0;
////                            element_found = true;
////                            break;
//                        }
//                    }
////                    if(element_found = true)
////                        break;
//                }
//            }
//        }
//
//        ROS_INFO("Matrix with threshold :");
//        // Display begin matrix state.
//	for ( int new_row = 0 ; new_row < mat.rows(); new_row++ ) {
//		for ( int new_col = 0 ; new_col < mat.columns(); new_col++ ) {
//			std::cout.width(2);
//			std::cout << mat(new_row,new_col) << " ";
//		}
//		std::cout << std::endl;
//	}
//	std::cout << std::endl;


	// Apply Munkres algorithm to matrix.
	Munkres munk;
	munk.solve(mat);

        ROS_INFO("Solved :");
	// Display solved matrix.
	for ( int new_row = 0 ; new_row < mat.rows(); new_row++ ) {
            if(new_row > 9)
            {
                int rows_left = mat.rows() - new_row + 1;
                std::cout << "... (" << rows_left << " more)";
                break;
            }
            for ( int new_col = 0 ; new_col < mat.columns(); new_col++ ) {
                if(new_col > 9)
                {
                    int columns_left = mat.columns() - new_col + 1;
                    std::cout << "... (" << columns_left << " more)";
                    break;
                }
                std::cout.width(2);
                std::cout << mat(new_row,new_col) << " ";
            }
            std::cout << std::endl;
	}

	std::cout << std::endl;


       own_row_to_select_cluster = row-1;
       for(int i = 0; i < mat.columns(); i++)
       {
           if(mat(i,own_row_to_select_cluster) == 1)
           {
               auction_cluster_element_id = clusters.at(i).id;
               own_column_to_select = i;
               ROS_INFO("Selected Cluster at position : %d   %d  with BID: %f",i ,own_row_to_select_cluster, mat(i, own_row_to_select_cluster));
               break;
           }
       }




    }





    /*
     * Try to navigate to the selected cluster. If it failes, take the next efficient
     * cluster and try again
     */


    if(auction_cluster_element_id != -1)
    {

       /*
        * Select the above calculated goal. If this cluster is still in use,
        * set this cluster in the matrix to zero and restart the auction
        */





        /*
         * Following should just be executed if the selection using auctioning
         * does fail
         */
        while(ros::ok())
        {
            ROS_INFO("Try to determining goal");
            std::vector<double> new_goal;
            std::vector<std::string> robot_str_name;
            bool goal_determined = determine_goal(6, &new_goal, count, auction_cluster_element_id, &robot_str_name);



            if(goal_determined == true)
            {
//                if(new_goal.front() == -1)
//                {
//                    ROS_INFO("Unable to access goal points in the cluster");
//                    count++;
//                }else
//                {
                    double determined_goal_id = new_goal.at(4);
                    bool used_cluster = false;
                    for(int m = 0; m < already_used_ids.size(); m++)
                    {
                        if(determined_goal_id == already_used_ids.at(m))
                        {
                            ROS_INFO("Cluster in use already");
                            cluster_in_use_already_count->push_back(1);
                            used_cluster = true;
                            count++;
                            break;
                        }
                    }
                    if(used_cluster == false)
                    {
                        ROS_INFO("Cluster was not used by any robot before");
                        goal->push_back(new_goal.at(0));
                        goal->push_back(new_goal.at(1));
                        goal->push_back(new_goal.at(2));
                        goal->push_back(new_goal.at(3));
                        goal->push_back(new_goal.at(4));

                        robot_str_name_to_return->push_back(robot_str_name.at(0));
                        return true;
                    }
//                }
            }else
            {
                if(clusters.size() <= count)
                {
                    ROS_INFO("Not possible to select any goal from the available clusters");
                    return false;
                }else
                {
                    ROS_INFO("Current cluster empty ... select next one");
                }
                count++;
            }
        }
        return false;
    }else
    {
        /*
         * No auction elements are available. Auctioning has failed
         */

        ROS_INFO("Auction has failed, no auction elements are existent. Choosing nearest cluster");
        std::vector<double> new_goal;
        std::vector<std::string> robot_str_name;
        bool goal_determined = determine_goal(4, &new_goal, count, -1, &robot_str_name);
        if(goal_determined == true)
        {
            goal->push_back(new_goal.at(0));
            goal->push_back(new_goal.at(1));
            goal->push_back(new_goal.at(2));
            goal->push_back(new_goal.at(3));
            goal->push_back(new_goal.at(4));
            return true;
        }else
        {
            return false;
        }
    }
    return false;
}

bool ExplorationPlanner::clusterIdToElementIds(int cluster_id, std::vector<transform_point_t>* occupied_ids)
{
    for(int i = 0; i < clusters.size(); i++)
    {
        if(clusters.at(i).id == cluster_id)
        {
            for(int j = 0; j < clusters.at(i).cluster_element.size(); j++)
            {
                transform_point_t point;
                point.id = clusters.at(i).cluster_element.at(j).id;
                point.robot_str = clusters.at(i).cluster_element.at(j).detected_by_robot_str;

                occupied_ids->push_back(point);
            }
        }
    }
}

double ExplorationPlanner::frontier_cost_1(frontier_t *frontier) {
    /*
     * cost function
     * f = w1 · d_g   +   w2 · d_gb   +   w3 · d_gbe   +   w4 · theta
     *
     * parameters
     * w1, ..., w4 .. weights
     * d_g         .. euclidean distance from the robot's current position to the frontier
     * d_gbe       .. -d_gb if battery charge is above threshold (e.g. 50%), d_gb if battery charge is below threshold,
     *                where d_gb is the euclidean distance from the frontier to the charging station
     * d_r         .. (MISSING DESCRIPTION) //TODO                 
     * theta       .. measure of how much the robot has to turn to reach frontier, theta in [0,1]
     */


    // frontier position
    double frontier_x = frontier->x_coordinate;
    double frontier_y = frontier->y_coordinate;

    // calculate d_g
    double d_g = frontier->my_distance_to_robot;

    // calculate d_gbe
    double d_gbe;
    {
        double d_gb;
        d_gb = frontier->my_distance_to_optimal_ds;

        if(my_energy_above_th)
        {
            d_gbe = -d_gb;
        }
        else
        {
            d_gbe = d_gb;
        }
    }
    
    // calculate d_r
    double d_r = 0;
    ros::Time time_now = ros::Time::now();
    for(unsigned int i=0; i<last_robot_auctioned_frontier_list.size(); i++) {
        if(time_now - last_robot_auctioned_frontier_list.at(i).timestamp > ros::Duration(VALIDITY_INTERVAL)) {
            ROS_INFO("expired");
            last_robot_auctioned_frontier_list.erase(last_robot_auctioned_frontier_list.begin() + i);
        }
        else {
            double distance = trajectory_plan_meters(frontier_x, frontier_y, last_robot_auctioned_frontier_list.at(i).x_coordinate, last_robot_auctioned_frontier_list.at(i).y_coordinate);
            if(distance < 0) {
                ROS_ERROR("failed distance");
                continue;
            }
            if(distance < d_r || d_r == 0) 
                d_r = distance;
        }
    }
    d_r = -d_r;

    // calculate theta
    double theta;
    if(use_theta)
        theta = computeTheta(frontier_x, frontier_y);
    else
        theta = 0;
    //DEBUGGING
    frontier->_theta = theta;

    // calculate cost function
    return w1 * d_g + w2 * d_gbe + w3 * d_r + w4 * theta;
 
}

//bool ExplorationPlanner::my2_determine_goal_staying_alive(int mode, int strategy, double available_distance, std::vector<double> *final_goal, int count, std::vector<std::string> *robot_str_name, int actual_cluster_id, bool energy_above_th, int w1, int w2, int w3, int w4)
//{
//    
//    ros::Time start_time;
//    sort_time = 0;
//    selection_time = 0;
//    number_of_frontiers = 0;
//    frontier_selected = false;
//    this->available_distance = available_distance;

//    if (frontiers.size() <= 0 && clusters.size() <= 0)
//    {
//        ROS_ERROR("No frontier/cluster available");
//    } 
//    
//    //if(APPROACH == 0)
//        /* Original cost function */
//    //    smart_sort_cost(energy_above_th, w1, w2, w3, w4);
//    //else if(APPROACH == 1)
//        /* Cost function: (real distance = Disjktra's distance)
//         *     - (real) distance between the given frontier and target DS of the robot;
//         *     - distance on the graph of the DSs between the DS that is closest, according to the euclidean distance (not the real one!), to the given frontier and the robot; this distance is computed efficiently by the energy_mgmt node;
//         *     - d_r
//         *     - theta_rel
//         */
//    //    sort_cost_1(energy_above_th, w1, w2, w3, w4);
//    //else if(APPROACH == -1)
//    //    return false;
//    //else
//    //    ROS_ERROR("INVALID APPROACH!!!");
//    
//    sorted_frontiers.clear();
//    
//    start_time = ros::Time::now();
//    number_of_frontiers = frontiers.size();
//    
//    //TODO move to a separate function that is called by explorer, since in case of error (when my_... is recalled by itself), this code otherwise is re-executed every time...
//    ROS_DEBUG("frontiers size: %u", frontiers.size());
//    if(APPROACH == 0)
//        sorted_frontiers = frontiers;
//    else if(APPROACH == 1)
//        sorted_frontiers = frontiers;
//    else if(APPROACH == 2)
//        my_sort_cost_2(energy_above_th, w1, w2, w3, w4);
//    else if(APPROACH == 3)
//        my_sort_cost_3(energy_above_th, w1, w2, w3, w4);
//    else if(APPROACH == 4)
//        my_sort_cost_4(energy_above_th, w1, w2, w3, w4);
//    else {
//        ROS_ERROR("Invalid approach!");
//        sorted_frontiers = frontiers;
//    }
//    
//    //return determine_goal_staying_alive( mode,  strategy,  available_distance, final_goal,  count, robot_str_name,  actual_cluster_id);
//    //sorted_frontiers = frontiers;

//    if (!costmap_ros_->getRobotPose(robotPose)) 
//    {
//            ROS_ERROR("Failed to get RobotPose"); //TODO handle "exception"
//    }
//    
//    if(sorted_frontiers.size() == 0) {
//        my_error_counter = 0;
//        //force to conciser also frontiers under auction (if there are)
//        for(int i=0; i < frontiers_under_auction.size(); i++)
//            sorted_frontiers.push_back(frontiers_under_auction.at(i));
//    }
//    
//    sort_time = (ros::Time::now() - start_time).toSec();
//    start_time = ros::Time::now();
//    
//    //store_frontier_mutex.lock();
//    acquire_mutex(&store_frontier_mutex, __FUNCTION__);

//    // check if robot needs to go recharging right away
//    double dist_home;
//    double dist_front;
//    double closest = 9999;
//    if(strategy == 1){
//        double xh = optimal_ds_x - robotPose.getOrigin().getX();
//        double yh = optimal_ds_y - robotPose.getOrigin().getY();
//        dist_home = sqrt(xh * xh + yh * yh) * 1.2; // 1.2 is extra reserve, just in case
//    }
//    else if(strategy == 2){
//        dist_home = trajectory_plan_meters(optimal_ds_x, optimal_ds_y); // * costmap_ros_->getCostmap()->getResolution();
//    }
//    //ROS_ERROR("available_distance: %f", available_distance);
//    if(dist_home > 0 && dist_home >= available_distance) {
//        ROS_ERROR("Target DS is too far to reach a frontier...\noptimal_ds_x: %f, optimal_ds_y: %f, distance: %f, available distance: %f", optimal_ds_x, optimal_ds_y, dist_home, available_distance);
//        release_mutex(&store_frontier_mutex, __FUNCTION__);
//        return false;
//    }

//    ROS_INFO("look for a FRONTIER as goal");
//    // look for a FRONTIER as goal
//    int errors = 0;
//    if (mode == 1)
//    {
//        //for (int i = 0 + count; i < sorted_frontiers.size(); i++)
//        ROS_DEBUG("sorted_frontiers size: %u", sorted_frontiers.size());
//        for (int i = 0; i < sorted_frontiers.size(); i++)
//        {
//            //we only check the first 9 frontiers, because we only sorted the first 9 frontiers by efficiency.
//            if(i>limit_search){
//                // if the distances could not be computed, try again using euclidean distances instead
//                //if(errors == i && strategy == 2){
//                if(errors >= 10 && strategy == 2){
//                    ROS_ERROR("Fallback to euclidean distance.");
//                    ROS_INFO("Fallback to euclidean distance.");
//                    release_mutex(&store_frontier_mutex, __FUNCTION__);
//                    selection_time = (ros::Time::now() - start_time).toSec();
//                    frontier_selected=my2_determine_goal_staying_alive(1, 1, available_distance, final_goal, count, robot_str_name, -1, energy_above_th, w1, w2, w3, w4);
//                    return frontier_selected;
//                }
//                ROS_ERROR("None of the %d checked frontiers is reachable! This shouldn't happen...", limit_search);
//                release_mutex(&store_frontier_mutex, __FUNCTION__);
//                frontier_selected=false;
//                selection_time = (ros::Time::now() - start_time).toSec();
//                return false;
//            }
//         
////            bool under_auction = false;
////            for(int k=0; !under_auction && k<frontiers_under_auction.size(); k++)
////                if(frontiers[i].id == frontiers_under_auction[k].id) {
////                    under_auction = true;
////                }
////            if(under_auction)
////                continue;

//            if (check_efficiency_of_goal(frontiers.at(i).x_coordinate, frontiers.at(i).y_coordinate) == true)
//            {
//                double distance;
//                double total_distance;
//                if(strategy == 1){
//                    // distance to next frontier
//                    double x1 = frontiers.at(i).x_coordinate - robotPose.getOrigin().getX();
//                    double y1 = frontiers.at(i).y_coordinate -  robotPose.getOrigin().getY();
//                    // distance from frontier to home base
//                    double x2 = optimal_ds_x - frontiers.at(i).x_coordinate;
//                    double y2 = optimal_ds_y -  frontiers.at(i).y_coordinate;
//                    total_distance = sqrt(x1 * x1 + y1 * y1) + sqrt(x2 * x2 + y2 * y2);
//                }
//                else if(strategy == 2){
//                    // distance to next frontier
//                    total_distance = trajectory_plan_meters(frontiers.at(i).x_coordinate, frontiers.at(i).y_coordinate);
//                    if(total_distance < 0){
//                        ROS_ERROR("Failed to compute distance!");
//                        ROS_INFO("Failed to compute distance!");
//                        if(errors == 0)
//                            my_error_counter++;
//                        errors++;
//                        continue;
//                    }
//                    // distance from frontier to optimal ds
//                    distance = trajectory_plan_meters(frontiers.at(i).x_coordinate, frontiers.at(i).y_coordinate, optimal_ds_x, optimal_ds_y);
//                    if(distance < 0){
//                        ROS_ERROR("Failed to compute distance!");
//                        ROS_INFO("Failed to compute distance!");
//                        if(errors == 0)
//                            my_error_counter++;
//                        errors++;
//                        continue;
//                    }
//                    total_distance += distance;

//                    if(total_distance < closest){
//                        closest = total_distance;
//                        dist_front = total_distance - distance;
//                        dist_home = distance;
//                    }

//                    // convert from cells to meters
//                    //F
//                    //total_distance *= costmap_ros_->getCostmap()->getResolution();
//                }
//                else{
//                    ROS_ERROR("Wrong strategy, cannot compute distance to goal!");
//                    release_mutex(&store_frontier_mutex, __FUNCTION__);
//                    frontier_selected=false;
//                    selection_time = (ros::Time::now() - start_time).toSec();
//                    return false;
//                }

//                ROS_INFO("Distance to frontier and then home: %.2f",total_distance);
//                if(available_distance > total_distance)
//                {
//                    ROS_INFO("------------------------------------------------------------------");
//                    ROS_INFO("Determined frontier with ID: %d   at x: %.2f     y: %.2f   detected by Robot %d", frontiers.at(i).id, frontiers.at(i).x_coordinate, frontiers.at(i).y_coordinate, frontiers.at(i).detected_by_robot);

//                    final_goal->push_back(frontiers.at(i).x_coordinate);
//                    final_goal->push_back(frontiers.at(i).y_coordinate);
//                    final_goal->push_back(frontiers.at(i).detected_by_robot);
//                    final_goal->push_back(frontiers.at(i).id);
//                    
//                    robot_str_name->push_back(frontiers.at(i).detected_by_robot_str);
//                    
//#ifndef QUICK_SELECTION
//            
//                    // robot position
//                    double robot_x = robotPose.getOrigin().getX();
//                    double robot_y = robotPose.getOrigin().getY();

//					// frontier position
//                    double frontier_x = frontiers.at(i).x_coordinate;
//                    double frontier_y = frontiers.at(i).y_coordinate;

//					// calculate d_g
//                    int d_g = trajectory_plan_meters(frontier_x, frontier_y);

//                    // calculate d_gb
//                    int d_gb = trajectory_plan_meters(frontier_x, frontier_y, robot_home_x, robot_home_y);

//                    // calculate d_gbe
//                    int d_gbe;
//                    if(my_energy_above_th)
//                    {
//                        d_gbe = -d_gb;
//                    }
//                    else
//                    {
//                        d_gbe = d_gb;
//                    }

//                    // calculate theta
//                    double theta_s = atan2(robot_last_y - robot_y, robot_last_x - robot_x);
//                    double theta_g = atan2(robot_y - frontier_y, robot_x - frontier_x);
//                    double theta = 1/M_PI * (M_PI - abs(abs(theta_s - theta_g) - M_PI));

//                    // calculate cost function
//                     my_bid = w1 * d_g + w2 * d_gb + w3 * d_gbe + w4 * theta;
//             
//#endif

//                    //start auction
//                    ROS_INFO("start frontier negotiation!");
//                    my_selected_frontier = &frontiers.at(i);
//                    //my_negotiate();
//             
//                    for(int i = 0; i < auction_timeout/0.1; i++) {
//                        ros::Duration(0.1).sleep();
//                        ros::spinOnce();
//                    }
//                    
//                    winner_of_auction = true;
//                    if(!winner_of_auction) {
//                        ROS_INFO("frontier under auction: skip");
//                        continue;
//                    }

//                    ROS_INFO("frontier selected");
//                    frontiers_under_auction.clear();
//                    release_mutex(&store_frontier_mutex, __FUNCTION__);
//                    my_error_counter = 0;
//                    //ROS_INFO("final_goal size before return: %u", final_goal->size());
//                    frontier_selected=true;
//                    selection_time = (ros::Time::now() - start_time).toSec();
//                    return true;
//                    
//                } else{
//                    //ROS_ERROR("No frontier in energetic range (%.2f < %.2f + %.2f)", available_distance, dist_front * costmap_ros_->getCostmap()->getResolution(), dist_home * costmap_ros_->getCostmap()->getResolution());
//                    ROS_ERROR("No frontier in energetic range (%.2f < %.2f + %.2f)", available_distance, dist_front // * costmap_ros_->getCostmap()->getResolution(),
//                    , dist_home);
//                    release_mutex(&store_frontier_mutex, __FUNCTION__);
//                    my_error_counter++;
//                    frontier_selected=false;
//                    selection_time = (ros::Time::now() - start_time).toSec();
//                    return false;
//                }
//            }
//        }
//    }

//    // look for a CLUSTER as goal
//    else if (mode == 2)
//    {
//        int cluster_vector_position = 0;

//        if(clusters.size() > 0)
//        {
//            for (int i = 0; i < clusters.size(); i++)
//            {
//                if(clusters.at(i).id == actual_cluster_id)
//                {
//                    if(clusters.at(i).cluster_element.size() > 0)
//                    {
//                        cluster_vector_position = i;
//                    }
//                    break;
//                }
//            }
//        }

//        ROS_INFO("Calculated vector position of cluster %d", actual_cluster_id);
//        /*
//         * Iterate over all clusters .... if cluster_vector_position is set
//         * also the clusters with a lower have to be checked if the frontier
//         * determination fails at clusters.at(cluster_vector_position). therefore
//         * a ring-buffer is operated to iterate also over lower clusters, since
//         * they might have changed.
//         */
//        int nothing_found_in_actual_cluster = 0;
//        int visited_clusters = 0;
//        for (int i = 0 + count; i < clusters.size(); i++)
//        {
//            i = i+ cluster_vector_position;
//            i = i % (clusters.size());
//            for (int j = 0; j < clusters.at(i).cluster_element.size(); j++)
//            {
//                if (check_efficiency_of_goal(clusters.at(i).cluster_element.at(j).x_coordinate, clusters.at(i).cluster_element.at(j).y_coordinate) == true)
//                {
//                    double distance;
//                    double total_distance;
//                    if(strategy == 1){
//                        // distance to cluster
//                        double x1 = clusters.at(i).cluster_element.at(j).x_coordinate - robotPose.getOrigin().getX();
//                        double y1 = clusters.at(i).cluster_element.at(j).y_coordinate - robotPose.getOrigin().getY();
//                        // distance from cluster to home base
//                        double x2 = robot_home_x - clusters.at(i).cluster_element.at(j).x_coordinate;
//                        double y2 = robot_home_y -  clusters.at(i).cluster_element.at(j).y_coordinate;
//                        total_distance = sqrt(x1 * x1 + y1 * y1) + sqrt(x2 * x2 + y2 * y2);
//                    }
//                    else if(strategy == 2){
//                        // distance to cluster
//                        total_distance = trajectory_plan_meters(clusters.at(i).cluster_element.at(j).x_coordinate, clusters.at(i).cluster_element.at(j).y_coordinate);
//                        if(total_distance < 0){
//                            ROS_ERROR("Failed to compute distance!");
//                            errors++;
//                            continue;
//                        }
//                        // distance from cluster to home base
//                        distance = trajectory_plan_meters(clusters.at(i).cluster_element.at(j).x_coordinate, clusters.at(i).cluster_element.at(j).y_coordinate, robot_home_x, robot_home_y);
//                        if(distance < 0){
//                            ROS_ERROR("Failed to compute distance!");
//                            errors++;
//                            continue;
//                        }
//                        total_distance += distance;
//                        // convert from cells to meters
//                        //total_distance *= costmap_ros_->getCostmap()->getResolution();
//                    }
//                    else{
//                        ROS_ERROR("Wrong strategy, cannot compute distance to goal!");
//                        release_mutex(&store_frontier_mutex, __FUNCTION__);
//                        frontier_selected=false;
//                        selection_time = (ros::Time::now() - start_time).toSec();
//                        return false;
//                    }

//                    ROS_INFO("distance to cluster and then home: %f",total_distance);
//                    if(available_distance > total_distance)
//                    {
//                        ROS_INFO("------------------------------------------------------------------");
//                        ROS_INFO("Robot %d determined Goal: %d  at Clusters: %d",robot_name, (int)clusters.at(i).cluster_element.at(j).id, (int)clusters.at(i).id);

//                        final_goal->push_back(clusters.at(i).cluster_element.at(j).x_coordinate);
//                        final_goal->push_back(clusters.at(i).cluster_element.at(j).y_coordinate);
//                        final_goal->push_back(clusters.at(i).cluster_element.at(j).detected_by_robot);
//                        final_goal->push_back(clusters.at(i).cluster_element.at(j).id);

//                        // number of the cluster we operate in
//                        final_goal->push_back(clusters.at(i).id);
//                        robot_str_name->push_back(clusters.at(i).cluster_element.at(j).detected_by_robot_str);
//                        release_mutex(&store_frontier_mutex, __FUNCTION__);
//                        frontier_selected=true;
//                        selection_time = (ros::Time::now() - start_time).toSec();
//                        return true;
//                    }
//                }
//            }

//            nothing_found_in_actual_cluster ++;
//            visited_clusters ++;

//            if(nothing_found_in_actual_cluster == 1)
//            {
//                //start again at the beginning(closest cluster))
//                i=0;
//                cluster_vector_position = 0;
//            }

//            if(visited_clusters == clusters.size())
//            {
//                ROS_ERROR("No frontier in energetic range %.2f, going home for recharging", available_distance);
//                release_mutex(&store_frontier_mutex, __FUNCTION__);
//                frontier_selected=false;
//                selection_time = (ros::Time::now() - start_time).toSec();
//                return false;
//            }
//        }
//    }
//    
//    //F //probably useless here...
//    release_mutex(&store_frontier_mutex, __FUNCTION__);
//    frontier_selected=false;
//    selection_time = (ros::Time::now() - start_time).toSec();
//    return false;

//}

int ExplorationPlanner::backoff (int point)
{
    				if(occupancy_grid_array_[up(point)] == costmap_2d::NO_INFORMATION && occupancy_grid_array_[left(point)] == costmap_2d::NO_INFORMATION)
				{
					return(downright(downright(downright(point))));
				}
				if(occupancy_grid_array_[up(point)] == costmap_2d::NO_INFORMATION && occupancy_grid_array_[right(point)] == costmap_2d::NO_INFORMATION)
				{
					return(downleft(downleft(downleft(point))));
				}
				if(occupancy_grid_array_[down(point)] == costmap_2d::NO_INFORMATION && occupancy_grid_array_[left(point)] == costmap_2d::NO_INFORMATION)
				{
					return(upright(upright(upright(point))));
				}
				if(occupancy_grid_array_[down(point)] == costmap_2d::NO_INFORMATION && occupancy_grid_array_[right(point)] == costmap_2d::NO_INFORMATION)
				{
					return(upleft(upleft(upleft(point))));
				}


				if(occupancy_grid_array_[right(point)] == costmap_2d::NO_INFORMATION)
				{
					return(left(left(left(point))));
				}
				if(occupancy_grid_array_[down(point)] == costmap_2d::NO_INFORMATION)
				{
					return(up(up(up(point))));
				}
				if(occupancy_grid_array_[left(point)] == costmap_2d::NO_INFORMATION)
				{
					return(right(right(right(point))));
				}
				if(occupancy_grid_array_[up(point)] == costmap_2d::NO_INFORMATION)
				{
					return(down(down(down(point))));
				}

//				if(occupancy_grid_array_[upleft(point)] == costmap_2d::NO_INFORMATION)
//				{
//					return(downright(downright(point)));
//				}
//				if(occupancy_grid_array_[upright(point)] == costmap_2d::NO_INFORMATION)
//				{
//					return(downleft(downleft(point)));
//				}
//				if(occupancy_grid_array_[downright(point)] == costmap_2d::NO_INFORMATION)
//				{
//					return(upleft(upleft(point)));
//				}
//				if(occupancy_grid_array_[downleft(point)] == costmap_2d::NO_INFORMATION)
//				{
//					return(upright(upright(point)));
//				}
				else
				{
					return(point);
				}
}

// Used to generate direction for frontiers
double ExplorationPlanner::getYawToUnknown(int point) {
	int adjacentPoints[8];
	getAdjacentPoints(point, adjacentPoints);

	int max_obs_idx = 0;

	for (int i = 0; i < 8; ++i) {
		if (isValid(adjacentPoints[i])) {
			if (occupancy_grid_array_[adjacentPoints[i]]
					== costmap_2d::NO_INFORMATION) {
				if (obstacle_trans_array_[adjacentPoints[i]]
						> obstacle_trans_array_[adjacentPoints[max_obs_idx]]) {
					max_obs_idx = i;
				}
			}
		}
	}

	int orientationPoint = adjacentPoints[max_obs_idx];
	unsigned int sx, sy, gx, gy;
	costmap_.indexToCells((unsigned int) point, sx, sy);
	costmap_.indexToCells((unsigned int) orientationPoint, gx, gy);
	int x = gx - sx;
	int y = gy - sy;
	double yaw = std::atan2(y, x);

	return yaw;

}


bool ExplorationPlanner::isSameFrontier(int frontier_point1, int frontier_point2) {
	unsigned int fx1, fy1;
	unsigned int fx2, fy2;
	double wfx1, wfy1;
	double wfx2, wfy2;
	costmap_.indexToCells(frontier_point1, fx1, fy1);
	costmap_.indexToCells(frontier_point2, fx2, fy2);
	costmap_.mapToWorld(fx1, fy1, wfx1, wfy1);
	costmap_.mapToWorld(fx2, fy2, wfx2, wfy2);

	double dx = wfx1 - wfx2;
	double dy = wfy1 - wfy2;

	if ((dx * dx) + (dy * dy)
			< (p_same_frontier_dist_ * p_same_frontier_dist_)) {
		return true;
	}
	return false;
}

void ExplorationPlanner::resetMaps() {
	std::fill_n(exploration_trans_array_, num_map_cells_, INT_MAX);
	std::fill_n(obstacle_trans_array_, num_map_cells_, INT_MAX);
	std::fill_n(is_goal_array_, num_map_cells_, false);
}

void ExplorationPlanner::deleteMapData() {
	if (exploration_trans_array_) {
		delete[] exploration_trans_array_;
		exploration_trans_array_ = 0;
	}
	if (obstacle_trans_array_) {
		delete[] obstacle_trans_array_;
		obstacle_trans_array_ = 0;
	}
	if (is_goal_array_) {
		delete[] is_goal_array_;
		is_goal_array_ = 0;
	}
	if (frontier_map_array_) {
		delete[] frontier_map_array_;
		frontier_map_array_ = 0;
	}
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
void ExplorationPlanner::visualize_Clusters()
{
    ROS_INFO("Visualize clusters");

    geometry_msgs::PolygonStamped cluster_polygon;

    for(int i= 0; i< clusters.size(); i++)
    {
        double upper_left_x, upper_left_y;
        double upper_right_x, upper_right_y;
        double down_left_x, down_left_y;
        double down_right_x, down_right_y;
        double most_left, most_right, upper1, lower1, upper2, lower2;

        for(int j= 0; j < clusters.at(i).cluster_element.size(); j++)
        {
            if(clusters.at(i).cluster_element.at(j).x_coordinate < most_left)
            {
                most_left = clusters.at(i).cluster_element.at(j).x_coordinate;
                if(clusters.at(i).cluster_element.at(j).y_coordinate < lower1)
                {
                    lower1 = clusters.at(i).cluster_element.at(j).y_coordinate;
                    down_left_x = clusters.at(i).cluster_element.at(j).x_coordinate;
                    down_left_y = clusters.at(i).cluster_element.at(j).y_coordinate;
                }
                if(clusters.at(i).cluster_element.at(j).y_coordinate > upper1)
                {
                    upper1 = clusters.at(i).cluster_element.at(j).y_coordinate;
                    upper_left_x = clusters.at(i).cluster_element.at(j).x_coordinate;
                    upper_left_y = clusters.at(i).cluster_element.at(j).y_coordinate;
                }
            }
            if(clusters.at(i).cluster_element.at(j).x_coordinate > most_right)
            {
                most_right = clusters.at(i).cluster_element.at(j).x_coordinate;
                if(clusters.at(i).cluster_element.at(j).y_coordinate < lower2)
                {
                    lower2 = clusters.at(i).cluster_element.at(j).y_coordinate;
                    down_right_x = clusters.at(i).cluster_element.at(j).x_coordinate;
                    down_right_y = clusters.at(i).cluster_element.at(j).y_coordinate;
                }
                if(clusters.at(i).cluster_element.at(j).y_coordinate > upper2)
                {
                    upper2 = clusters.at(i).cluster_element.at(j).y_coordinate;
                    upper_right_x = clusters.at(i).cluster_element.at(j).x_coordinate;
                    upper_right_y = clusters.at(i).cluster_element.at(j).y_coordinate;
                }
            }

        }

        geometry_msgs::Point32 polygon_point;
        polygon_point.x = upper_left_x;
        polygon_point.y = upper_left_y;
        polygon_point.z = 0;
        cluster_polygon.polygon.points.push_back(polygon_point);

        polygon_point.x = upper_right_x;
        polygon_point.y = upper_right_y;
        polygon_point.z = 0;
        cluster_polygon.polygon.points.push_back(polygon_point);

        polygon_point.x = down_left_x;
        polygon_point.y = down_left_y;
        polygon_point.z = 0;
        cluster_polygon.polygon.points.push_back(polygon_point);

        polygon_point.x = down_right_x;
        polygon_point.y = down_right_y;
        polygon_point.z = 0;
        cluster_polygon.polygon.points.push_back(polygon_point);

        cluster_polygon.header.frame_id = move_base_frame;
        cluster_polygon.header.stamp = ros::Time::now();
        cluster_polygon.header.seq = i+1;

        pub_clusters.publish<geometry_msgs::PolygonStamped>(cluster_polygon);
        //break;// FIXME
    }
}
#pragma GCC diagnostic pop

void ExplorationPlanner::visualize_Cluster_Cells()
{
    ROS_INFO("Visualize clusters");

    std::vector<int> used_ids;
    for(int i= 0; i< clusters.size(); i++)
    {
        if(clusters.at(i).cluster_element.size() > 0)
        {
            nav_msgs::GridCells cluster_cells;
            geometry_msgs::Point point;
            for(int j= 0; j < clusters.at(i).cluster_element.size(); j++)
            {
                point.x = clusters.at(i).cluster_element.at(j).x_coordinate;
                point.y = clusters.at(i).cluster_element.at(j).y_coordinate;
                point.z = 0;

                cluster_cells.cells.push_back(point);
            }
            cluster_cells.header.frame_id = move_base_frame;
            cluster_cells.header.stamp = ros::Time::now();
            cluster_cells.cell_height = 0.5;
            cluster_cells.cell_width = 0.5;
            cluster_cells.header.seq = cluster_cells_seq_number++;

            used_ids.push_back(clusters.at(i).id % 20);

            if(clusters.at(i).id % 20 == 0)
            {
                pub_cluster_grid_0.publish<nav_msgs::GridCells>(cluster_cells);
            }
            else if(clusters.at(i).id % 20 == 1)
            {
                pub_cluster_grid_1.publish<nav_msgs::GridCells>(cluster_cells);
            }
            else if(clusters.at(i).id % 20 == 2)
            {
                pub_cluster_grid_2.publish<nav_msgs::GridCells>(cluster_cells);
            }
            else if(clusters.at(i).id % 20 == 3)
            {
                pub_cluster_grid_3.publish<nav_msgs::GridCells>(cluster_cells);
            }
            else if(clusters.at(i).id % 20 == 4)
            {
                pub_cluster_grid_4.publish<nav_msgs::GridCells>(cluster_cells);
            }
            else if(clusters.at(i).id % 20 == 5)
            {
                pub_cluster_grid_5.publish<nav_msgs::GridCells>(cluster_cells);
            }
            else if(clusters.at(i).id % 20 == 6)
            {
                pub_cluster_grid_6.publish<nav_msgs::GridCells>(cluster_cells);
            }
            else if(clusters.at(i).id % 20 == 7)
            {
                pub_cluster_grid_7.publish<nav_msgs::GridCells>(cluster_cells);
            }
            else if(clusters.at(i).id % 20 == 8)
            {
                pub_cluster_grid_8.publish<nav_msgs::GridCells>(cluster_cells);
            }
            else if(clusters.at(i).id % 20 == 9)
            {
                pub_cluster_grid_9.publish<nav_msgs::GridCells>(cluster_cells);
            }

            if(clusters.at(i).id % 20 == 10)
            {
                pub_cluster_grid_10.publish<nav_msgs::GridCells>(cluster_cells);
            }
            else if(clusters.at(i).id % 20 == 11)
            {
                pub_cluster_grid_11.publish<nav_msgs::GridCells>(cluster_cells);
            }
            else if(clusters.at(i).id % 20 == 12)
            {
                pub_cluster_grid_12.publish<nav_msgs::GridCells>(cluster_cells);
            }
            else if(clusters.at(i).id % 20 == 13)
            {
                pub_cluster_grid_13.publish<nav_msgs::GridCells>(cluster_cells);
            }
            else if(clusters.at(i).id % 20 == 14)
            {
                pub_cluster_grid_14.publish<nav_msgs::GridCells>(cluster_cells);
            }
            else if(clusters.at(i).id % 20 == 15)
            {
                pub_cluster_grid_15.publish<nav_msgs::GridCells>(cluster_cells);
            }
            else if(clusters.at(i).id % 20 == 16)
            {
                pub_cluster_grid_16.publish<nav_msgs::GridCells>(cluster_cells);
            }
            else if(clusters.at(i).id % 20 == 17)
            {
                pub_cluster_grid_17.publish<nav_msgs::GridCells>(cluster_cells);
            }
            else if(clusters.at(i).id % 20 == 18)
            {
                pub_cluster_grid_18.publish<nav_msgs::GridCells>(cluster_cells);
            }
            else if(clusters.at(i).id % 20 == 19)
            {
                pub_cluster_grid_19.publish<nav_msgs::GridCells>(cluster_cells);
            }
        }
    }
    clear_Visualized_Cluster_Cells(used_ids);
}

void ExplorationPlanner::clear_Visualized_Cluster_Cells(std::vector<int> ids)
{
    nav_msgs::GridCells clearing_cell;
    geometry_msgs::Point point;

    clearing_cell.header.frame_id = move_base_frame;
    clearing_cell.header.stamp = ros::Time::now();
    clearing_cell.cell_height = 0.1;
    clearing_cell.cell_width = 0.1;
    clearing_cell.header.seq = cluster_cells_seq_number++;

    point.x = 1000;
    point.y = 1000;
    point.z = 1000;
    clearing_cell.cells.push_back(point);

    for(int i = 0; i < 20; i++)
    {
        bool used_id = false;
        for(int n = 0; n < ids.size(); n++)
        {
            if(ids.at(n) == i)
            {
               used_id = true;
               break;
            }
        }
        if(used_id == false)
        {
            if(i == 0)
            {
                pub_cluster_grid_0.publish<nav_msgs::GridCells>(clearing_cell);
            }
            if(i == 1)
            {
                pub_cluster_grid_1.publish<nav_msgs::GridCells>(clearing_cell);
            }
            if(i == 2)
            {
                pub_cluster_grid_2.publish<nav_msgs::GridCells>(clearing_cell);
            }
            if(i == 3)
            {
                pub_cluster_grid_3.publish<nav_msgs::GridCells>(clearing_cell);
            }
            if(i == 4)
            {
                pub_cluster_grid_4.publish<nav_msgs::GridCells>(clearing_cell);
            }
            if(i == 5)
            {
                pub_cluster_grid_5.publish<nav_msgs::GridCells>(clearing_cell);
            }
            if(i == 6)
            {
                pub_cluster_grid_6.publish<nav_msgs::GridCells>(clearing_cell);
            }
            if(i == 7)
            {
                pub_cluster_grid_7.publish<nav_msgs::GridCells>(clearing_cell);
            }
            if(i == 8)
            {
                pub_cluster_grid_8.publish<nav_msgs::GridCells>(clearing_cell);
            }
            if(i == 9)
            {
                pub_cluster_grid_9.publish<nav_msgs::GridCells>(clearing_cell);
            }

            if(i == 10)
            {
                pub_cluster_grid_10.publish<nav_msgs::GridCells>(clearing_cell);
            }
            if(i == 11)
            {
                pub_cluster_grid_11.publish<nav_msgs::GridCells>(clearing_cell);
            }
            if(i == 12)
            {
                pub_cluster_grid_12.publish<nav_msgs::GridCells>(clearing_cell);
            }
            if(i == 13)
            {
                pub_cluster_grid_13.publish<nav_msgs::GridCells>(clearing_cell);
            }
            if(i == 14)
            {
                pub_cluster_grid_14.publish<nav_msgs::GridCells>(clearing_cell);
            }
            if(i == 15)
            {
                pub_cluster_grid_15.publish<nav_msgs::GridCells>(clearing_cell);
            }
            if(i == 16)
            {
                pub_cluster_grid_16.publish<nav_msgs::GridCells>(clearing_cell);
            }
            if(i == 17)
            {
                pub_cluster_grid_17.publish<nav_msgs::GridCells>(clearing_cell);
            }
            if(i == 18)
            {
                pub_cluster_grid_18.publish<nav_msgs::GridCells>(clearing_cell);
            }
            if(i == 19)
            {
                pub_cluster_grid_19.publish<nav_msgs::GridCells>(clearing_cell);
            }
        }
    }
}

void ExplorationPlanner::simulate() {

	geometry_msgs::PointStamped goalPoint;


	tf::Stamped < tf::Pose > robotPose;
	if (!costmap_ros_->getRobotPose(robotPose))
	{
		ROS_ERROR("Failed to get RobotPose");
	}
	// Visualize in RVIZ

	for (int i = frontiers.size() - 1; i >= 0; i--) {
            goalPoint.header.seq = i + 1;
            goalPoint.header.stamp = ros::Time::now();
            goalPoint.header.frame_id = "map";
            goalPoint.point.x = frontiers.at(i).x_coordinate;
            goalPoint.point.y = frontiers.at(i).y_coordinate;

            pub_Point.publish < geometry_msgs::PointStamped > (goalPoint);

            ros::Duration(1.0).sleep();
	}
}

//void ExplorationPlanner::sort_reserve(int strategy)
//{
//    tf::Stamped < tf::Pose > robotPose;
//    if (!costmap_ros_->getRobotPose(robotPose))
//    {
//        ROS_ERROR("Failed to get RobotPose");
//        return;
//    }
//    if (frontiers.size() <= 0 && clusters.size() <= 0)
//    {
//        ROS_INFO("Sorting not possible, no frontiers available!!!");
//        return;
//    }

//    double pose_x = robotPose.getOrigin().getX();
//    double pose_y = robotPose.getOrigin().getY();
//    
//    //store_frontier_mutex.lock();
//    acquire_mutex(&store_frontier_mutex, __FUNCTION__);
//    
//    //ROS_ERROR("COPYING...");
//    sorted_frontiers.clear();
//    for(int i=0; i<frontiers.size(); i++)
//        sorted_frontiers.push_back(frontiers.at(i));
//    release_mutex(&store_frontier_mutex, __FUNCTION__);
//    //ROS_ERROR("SORTING...");

//    /*
//     * Following Sort algorithm normalizes all distances to the
//     * robots actual position. As result, the list is sorted
//     * from smallest to biggest deviation between goal point and
//     * robot!
//     */
//    if(strategy == 1)
//    {
//        for (int i = sorted_frontiers.size(); i >= 0; i--) {
//            for (int j = 0; j < sorted_frontiers.size() - 1; j++) {
//                double x = sorted_frontiers.at(j).x_coordinate - robotPose.getOrigin().getX();
//                double y = sorted_frontiers.at(j).y_coordinate - robotPose.getOrigin().getY();
//                double x_next = sorted_frontiers.at(j+1).x_coordinate - robotPose.getOrigin().getX();
//                double y_next = sorted_frontiers.at(j+1).y_coordinate - robotPose.getOrigin().getY();
//                double euclidean_distance = x * x + y * y;
//                double euclidean_distance_next = x_next * x_next + y_next * y_next;

//                if (sqrt(euclidean_distance) > sqrt(euclidean_distance_next)) {
//                    frontier_t temp = sorted_frontiers.at(j+1);
//                    sorted_frontiers.at(j + 1) = sorted_frontiers.at(j);
//                    sorted_frontiers.at(j) = temp;
//                }
//            }
//        }
//    }
//    else if(strategy == 2)
//    {
//        for (int i = sorted_frontiers.size(); i >= 0; i--)
//        {
//            for (int j = 0; j < sorted_frontiers.size() - 1; j++) {
//                double x = sorted_frontiers.at(j).x_coordinate - robotPose.getOrigin().getX();
//                double y = sorted_frontiers.at(j).y_coordinate - robotPose.getOrigin().getY();
//                double x_next = sorted_frontiers.at(j+1).x_coordinate - robotPose.getOrigin().getX();
//                double y_next = sorted_frontiers.at(j+1).y_coordinate - robotPose.getOrigin().getY();
//                double euclidean_distance = x * x + y * y;
//                double euclidean_distance_next = x_next * x_next + y_next * y_next;

//                if (sqrt(euclidean_distance) > sqrt(euclidean_distance_next)) {
//                    frontier_t temp = sorted_frontiers.at(j+1);
//                    sorted_frontiers.at(j + 1) = sorted_frontiers.at(j);
//                    sorted_frontiers.at(j) = temp;
//                }
//            }
//        }
//    }
//    else if(strategy == 3)
//    {
//        trajectory_plan_10_frontiers();

//        for (int i = sorted_frontiers.size(); i >= sorted_frontiers.size()-10; i--)
//        {
//            if(sorted_frontiers.size()-i >= sorted_frontiers.size())
//            {
//                break;
//            }
//            else
//            {
//                for (int j = 0; j < 10-1; j++)
//                {
//                    if(j >= sorted_frontiers.size())
//                    {
//                        break;
//                    }else
//                    {

//                        int dist = sorted_frontiers.at(j).distance_to_robot;
//                        int dist_next = sorted_frontiers.at(j+1).distance_to_robot;

//                        if (dist > dist_next) {
//                                frontier_t temp = sorted_frontiers.at(j+1);
//                                sorted_frontiers.at(j + 1) = sorted_frontiers.at(j);
//                                sorted_frontiers.at(j) = temp;
//                        }
//                    }
//                }
//            }
//        }
//    }
//    else if(strategy == 4)
//    {
//        for(int cluster_number = 0; cluster_number < clusters.size(); cluster_number++)
//        {
//            if (clusters.at(cluster_number).cluster_element.size() > 0)
//            {
//                ROS_DEBUG("Cluster %d  size: %u",cluster_number, clusters.at(cluster_number).cluster_element.size());
//                for (int i = clusters.at(cluster_number).cluster_element.size(); i > 0; i--)
//                {
//                    ROS_DEBUG("Cluster element size: %d", i);
//                        for (int j = 0; j < clusters.at(cluster_number).cluster_element.size()-1; j++)
//                        {
//                            ROS_DEBUG("Cluster element number: %d", j);
//                            double x = clusters.at(cluster_number).cluster_element.at(j).x_coordinate - pose_x;
//                            double y = clusters.at(cluster_number).cluster_element.at(j).y_coordinate - pose_y;
//                            double x_next = clusters.at(cluster_number).cluster_element.at(j+1).x_coordinate - pose_x;
//                            double y_next = clusters.at(cluster_number).cluster_element.at(j+1).y_coordinate - pose_y;
//                            double euclidean_distance = x * x + y * y;
//                            double euclidean_distance_next = x_next * x_next + y_next * y_next;

//                            if (sqrt(euclidean_distance) > sqrt(euclidean_distance_next))
//                            {
//                                frontier_t temp = clusters.at(cluster_number).cluster_element.at(j+1);
//                                clusters.at(cluster_number).cluster_element.at(j+1) = clusters.at(cluster_number).cluster_element.at(j);
//                                clusters.at(cluster_number).cluster_element.at(j) = temp;
//                            }
//                        }
//                    }
//            }else
//            {
//                ROS_INFO("Sorting not possible, no elements available!!!");
//            }
//        }
//    }
//    else if(strategy == 5)
//    {
//        ROS_DEBUG("Iterating over all cluster elements");

//        for(int i = 0; i < clusters.size(); i++)
//        {
//            for(int j = 0; j < clusters.at(i).cluster_element.size(); j++)
//            {
//                clusters.at(i).cluster_element.at(j).dist_to_robot = 0;
//            }
//        }

//        for(int i = 0; i < clusters.size(); i++)
//        {
//            if(clusters.at(i).cluster_element.size() > 0)
//            {
//                for(int j = 0; j < clusters.at(i).cluster_element.size(); j++)
//                {
//                    /* ********** EUCLIDEAN DISTANCE ********** */
//                    double x = clusters.at(i).cluster_element.at(j).x_coordinate - pose_x;
//                    double y = clusters.at(i).cluster_element.at(j).y_coordinate - pose_y;
//                    double euclidean_distance = x * x + y * y;
//                    int distance = euclidean_distance;

//                    clusters.at(i).cluster_element.at(j).dist_to_robot = sqrt(distance);
//                }
//            }else
//            {
//                ROS_DEBUG("Erasing Cluster: %d", clusters.at(i).id);
//                cluster_mutex.lock();
//                clusters.erase(clusters.begin() + i);
//                cluster_mutex.unlock();
//                if(i > 0)
//                {
//                    i --;
//                }
//            }
//        }
//        ROS_INFO("Starting to sort the clusters itself");
//        std::sort(clusters.begin(), clusters.end(), sortCluster);

//    }
//    else if(strategy == 6)
//    {
//        ROS_DEBUG("Iterating over all cluster elements");

//        for(int i = 0; i < clusters.size(); i++)
//        {
//            if(clusters.at(i).cluster_element.size() > 0)
//            {
//                for(int j = 0; j < clusters.at(i).cluster_element.size(); j++)
//                {
//                    random_value = int(rand() % 100);
//                    clusters.at(i).cluster_element.at(j).dist_to_robot = random_value;
//                }
//            }else
//            {
//                ROS_DEBUG("Erasing Cluster: %d", clusters.at(i).id);
//                clusters.erase(clusters.begin() + i);
//                if(i > 0)
//                {
//                    i --;
//                }
//            }
//        }
//        ROS_DEBUG("Starting to sort the clusters itself");
//        std::sort(clusters.begin(), clusters.end(), sortCluster);
//    }
//    else if(strategy == 7)
//    {
//        double x,y,x_next,y_next,angle_robot,angle_frontier,angle_next_frontier,angle,angle_next;
//        int costmap_width,costmap_height;
//        close_frontiers.clear();
//        far_frontiers.clear();

//        // get size of local costmap
//        nh.param<int>("local_costmap/width",costmap_width,-1);
//        nh.param<int>("local_costmap/height",costmap_height,-1);

//        // differentiate between frontiers inside (close frontiers) and outside (far frontiers) of local costmap
//        for (int i = 0; i < sorted_frontiers.size(); i++)
//        {
//            x = sorted_frontiers.at(i).x_coordinate - robotPose.getOrigin().getX();
//            y = sorted_frontiers.at(i).y_coordinate - robotPose.getOrigin().getY();
//            if (fabs(x) <= CLOSE_FRONTIER_RANGE && fabs(y) <= CLOSE_FRONTIER_RANGE){
//                close_frontiers.push_back(sorted_frontiers.at(i));
//            }
//            else{
//                far_frontiers.push_back(sorted_frontiers.at(i));
//                ROS_INFO("distance: (%.2f, %.2f) (far)", fabs(x), fabs(y));
//            }
//        }

//        // sort close frontiers clock wise
//        if (close_frontiers.size() > 0)
//        {
//            //std::sort(close_frontiers.begin(), close_frontiers.end(), *this);
//            for (int i = 0; i< close_frontiers.size(); i++)
//            {
//                for (int j = 0; j < close_frontiers.size() - 1; j++)
//                {
//                    angle_robot = robotPose.getRotation().getAngle();

//                    angle_frontier = atan2(robotPose.getOrigin().getX()-close_frontiers.at(j).x_coordinate, robotPose.getOrigin().getY()-close_frontiers.at(j).y_coordinate);
//                    angle_next_frontier = atan2(robotPose.getOrigin().getX()-close_frontiers.at(j+1).x_coordinate, robotPose.getOrigin().getY()-close_frontiers.at(j+1).y_coordinate);

//                    if (angle_frontier > angle_next_frontier)
//                    {
//                        frontier_t temp = close_frontiers.at(j+1);
//                        close_frontiers.at(j+1) = close_frontiers.at(j);
//                        close_frontiers.at(j) = temp;
//                    }
//                }
//            }
//        }

//        // sort far frontiers by distance
//        if (far_frontiers.size() > 0)
//        {
//            for (int i = 0; i < far_frontiers.size(); i++)
//            {
//                for (int j = 0; j < far_frontiers.size() - 1; j++)
//                {
//                    x = far_frontiers.at(j).x_coordinate - robotPose.getOrigin().getX();
//                    y = far_frontiers.at(j).y_coordinate - robotPose.getOrigin().getY();
//                    x_next = far_frontiers.at(j+1).x_coordinate - robotPose.getOrigin().getX();
//                    y_next = far_frontiers.at(j+1).y_coordinate - robotPose.getOrigin().getY();

//                    if (x*x + y*y > x_next*x_next + y_next*y_next) {
//                        frontier_t temp = far_frontiers.at(j+1);
//                        far_frontiers.at(j + 1) = far_frontiers.at(j);
//                        far_frontiers.at(j) = temp;
//                    }
//                }
//            }
//        }

//        // put together close and far frontiers
//        sorted_frontiers.clear();
//        sorted_frontiers.reserve(close_frontiers.size() + far_frontiers.size());
//        sorted_frontiers.insert(frontiers.end(), close_frontiers.begin(), close_frontiers.end());
//        sorted_frontiers.insert(frontiers.end(), far_frontiers.begin(), far_frontiers.end());
//    }

//    ROS_INFO("Done sorting");
//}

void ExplorationPlanner::sort(int strategy)
{
    tf::Stamped < tf::Pose > robotPose;
    if (!costmap_ros_->getRobotPose(robotPose))
    {
        ROS_ERROR("Failed to get RobotPose");
        return;
    }
    if (frontiers.size() <= 0 && clusters.size() <= 0)
    {
        ROS_INFO("Sorting not possible, no frontiers available!!!");
        return;
    }

    double pose_x = robotPose.getOrigin().getX();
    double pose_y = robotPose.getOrigin().getY();
    
    //store_frontier_mutex.lock();
    acquire_mutex(&store_frontier_mutex, __FUNCTION__);
    
    //for (int i = frontiers.size()-1; i >= 0; i--) {
    //    ROS_ERROR("frontier %d: (%f, %f)", i, frontiers.at(i).x_coordinate, frontiers.at(i).y_coordinate);
    //}

    /*
     * Following Sort algorithm normalizes all distances to the
     * robots actual position. As result, the list is sorted
     * from smallest to biggest deviation between goal point and
     * robot!
     */
    if(strategy == 1)
    {
        for (int i = frontiers.size(); i >= 0; i--) {
            for (int j = 0; j < frontiers.size() - 1; j++) {
                double x = frontiers.at(j).x_coordinate - robotPose.getOrigin().getX();
                double y = frontiers.at(j).y_coordinate - robotPose.getOrigin().getY();
                double x_next = frontiers.at(j+1).x_coordinate - robotPose.getOrigin().getX();
                double y_next = frontiers.at(j+1).y_coordinate - robotPose.getOrigin().getY();
                double euclidean_distance = x * x + y * y;
                double euclidean_distance_next = x_next * x_next + y_next * y_next;

                if (sqrt(euclidean_distance) > sqrt(euclidean_distance_next)) {
                    frontier_t temp = frontiers.at(j+1);
                    frontiers.at(j + 1) = frontiers.at(j);
                    frontiers.at(j) = temp;
                }
            }
        }
    }
    else if(strategy == 2)
    {
        for (int i = frontiers.size(); i >= 0; i--)
        {
            //ROS_ERROR("%d", i);
            for (int j = 0; j < frontiers.size() - 1; j++) {
                double x = frontiers.at(j).x_coordinate - robotPose.getOrigin().getX();
                double y = frontiers.at(j).y_coordinate - robotPose.getOrigin().getY();
                double x_next = frontiers.at(j+1).x_coordinate - robotPose.getOrigin().getX();
                double y_next = frontiers.at(j+1).y_coordinate - robotPose.getOrigin().getY();
                double euclidean_distance = x * x + y * y;
                double euclidean_distance_next = x_next * x_next + y_next * y_next;

                if (sqrt(euclidean_distance) > sqrt(euclidean_distance_next)) {
                    frontier_t temp = frontiers.at(j+1);
                    frontiers.at(j + 1) = frontiers.at(j);
                    frontiers.at(j) = temp;
                }
            }
        }
    }
    else if(strategy == 3)
    {
        trajectory_plan_10_frontiers();

        for (int i = frontiers.size(); i >= frontiers.size()-10; i--)
        {
            //ROS_ERROR("%d", i);
            if(frontiers.size()-i >= frontiers.size())
            {
                break;
            }
            else
            {
                for (int j = 0; j < 10-1; j++)
                {
                    if(j >= frontiers.size())
                    {
                        break;
                    }else
                    {

                        int dist = frontiers.at(j).distance_to_robot;
                        int dist_next = frontiers.at(j+1).distance_to_robot;

                        if (dist > dist_next) {
                                frontier_t temp = frontiers.at(j+1);
                                frontiers.at(j + 1) = frontiers.at(j);
                                frontiers.at(j) = temp;
                        }
                    }
                }
            }
        }
    }
    else if(strategy == 4)
    {
        for(int cluster_number = 0; cluster_number < clusters.size(); cluster_number++)
        {
            if (clusters.at(cluster_number).cluster_element.size() > 0)
            {
                ROS_DEBUG("Cluster %d  size: %u",cluster_number, clusters.at(cluster_number).cluster_element.size());
                for (int i = clusters.at(cluster_number).cluster_element.size(); i > 0; i--)
                {
                    ROS_DEBUG("Cluster element size: %d", i);
                        for (int j = 0; j < clusters.at(cluster_number).cluster_element.size()-1; j++)
                        {
                            ROS_DEBUG("Cluster element number: %d", j);
                            double x = clusters.at(cluster_number).cluster_element.at(j).x_coordinate - pose_x;
                            double y = clusters.at(cluster_number).cluster_element.at(j).y_coordinate - pose_y;
                            double x_next = clusters.at(cluster_number).cluster_element.at(j+1).x_coordinate - pose_x;
                            double y_next = clusters.at(cluster_number).cluster_element.at(j+1).y_coordinate - pose_y;
                            double euclidean_distance = x * x + y * y;
                            double euclidean_distance_next = x_next * x_next + y_next * y_next;

                            if (sqrt(euclidean_distance) > sqrt(euclidean_distance_next))
                            {
                                frontier_t temp = clusters.at(cluster_number).cluster_element.at(j+1);
                                clusters.at(cluster_number).cluster_element.at(j+1) = clusters.at(cluster_number).cluster_element.at(j);
                                clusters.at(cluster_number).cluster_element.at(j) = temp;
                            }
                        }
                    }
            }else
            {
                ROS_INFO("Sorting not possible, no elements available!!!");
            }
        }
    }
    else if(strategy == 5)
    {
        ROS_DEBUG("Iterating over all cluster elements");

        for(int i = 0; i < clusters.size(); i++)
        {
            for(int j = 0; j < clusters.at(i).cluster_element.size(); j++)
            {
                clusters.at(i).cluster_element.at(j).dist_to_robot = 0;
            }
        }

        for(int i = 0; i < clusters.size(); i++)
        {
            if(clusters.at(i).cluster_element.size() > 0)
            {
                for(int j = 0; j < clusters.at(i).cluster_element.size(); j++)
                {
                    /* ********** EUCLIDEAN DISTANCE ********** */
                    double x = clusters.at(i).cluster_element.at(j).x_coordinate - pose_x;
                    double y = clusters.at(i).cluster_element.at(j).y_coordinate - pose_y;
                    double euclidean_distance = x * x + y * y;
                    int distance = euclidean_distance;

                    clusters.at(i).cluster_element.at(j).dist_to_robot = sqrt(distance);
                }
            }else
            {
                ROS_DEBUG("Erasing Cluster: %d", clusters.at(i).id);
                cluster_mutex.lock();
                clusters.erase(clusters.begin() + i);
                cluster_mutex.unlock();
                if(i > 0)
                {
                    i --;
                }
            }
        }
        ROS_INFO("Starting to sort the clusters itself");
        std::sort(clusters.begin(), clusters.end(), sortCluster);

    }
    else if(strategy == 6)
    {
        ROS_DEBUG("Iterating over all cluster elements");

        for(int i = 0; i < clusters.size(); i++)
        {
            if(clusters.at(i).cluster_element.size() > 0)
            {
                for(int j = 0; j < clusters.at(i).cluster_element.size(); j++)
                {
                    random_value = int(rand() % 100);
                    clusters.at(i).cluster_element.at(j).dist_to_robot = random_value;
                }
            }else
            {
                ROS_DEBUG("Erasing Cluster: %d", clusters.at(i).id);
                clusters.erase(clusters.begin() + i);
                if(i > 0)
                {
                    i --;
                }
            }
        }
        ROS_DEBUG("Starting to sort the clusters itself");
        std::sort(clusters.begin(), clusters.end(), sortCluster);
    }
    else if(strategy == 7)
    {
        double x,y,x_next,y_next,angle_robot,angle_frontier,angle_next_frontier,angle,angle_next;
        int costmap_width,costmap_height;
        close_frontiers.clear();
        far_frontiers.clear();

        // get size of local costmap
        nh.param<int>("local_costmap/width",costmap_width,-1);
        nh.param<int>("local_costmap/height",costmap_height,-1);

        // differentiate between frontiers inside (close frontiers) and outside (far frontiers) of local costmap
        for (int i = 0; i < frontiers.size(); i++)
        {
            x = frontiers.at(i).x_coordinate - robotPose.getOrigin().getX();
            y = frontiers.at(i).y_coordinate - robotPose.getOrigin().getY();
            if (fabs(x) <= CLOSE_FRONTIER_RANGE && fabs(y) <= CLOSE_FRONTIER_RANGE){
                close_frontiers.push_back(frontiers.at(i));
            }
            else{
                far_frontiers.push_back(frontiers.at(i));
                ROS_INFO("distance: (%.2f, %.2f) (far)", fabs(x), fabs(y));
            }
        }

        // sort close frontiers clock wise
        if (close_frontiers.size() > 0)
        {
            //std::sort(close_frontiers.begin(), close_frontiers.end(), *this);
            for (int i = 0; i< close_frontiers.size(); i++)
            {
                for (int j = 0; j < close_frontiers.size() - 1; j++)
                {
                    angle_robot = robotPose.getRotation().getAngle();

                    angle_frontier = atan2(robotPose.getOrigin().getX()-close_frontiers.at(j).x_coordinate, robotPose.getOrigin().getY()-close_frontiers.at(j).y_coordinate);
                    angle_next_frontier = atan2(robotPose.getOrigin().getX()-close_frontiers.at(j+1).x_coordinate, robotPose.getOrigin().getY()-close_frontiers.at(j+1).y_coordinate);

                    if (angle_frontier > angle_next_frontier)
                    {
                        frontier_t temp = close_frontiers.at(j+1);
                        close_frontiers.at(j+1) = close_frontiers.at(j);
                        close_frontiers.at(j) = temp;
                    }
                }
            }
        }

        // sort far frontiers by distance
        if (far_frontiers.size() > 0)
        {
            for (int i = 0; i < far_frontiers.size(); i++)
            {
                for (int j = 0; j < far_frontiers.size() - 1; j++)
                {
                    x = far_frontiers.at(j).x_coordinate - robotPose.getOrigin().getX();
                    y = far_frontiers.at(j).y_coordinate - robotPose.getOrigin().getY();
                    x_next = far_frontiers.at(j+1).x_coordinate - robotPose.getOrigin().getX();
                    y_next = far_frontiers.at(j+1).y_coordinate - robotPose.getOrigin().getY();

                    if (x*x + y*y > x_next*x_next + y_next*y_next) {
                        frontier_t temp = far_frontiers.at(j+1);
                        far_frontiers.at(j + 1) = far_frontiers.at(j);
                        far_frontiers.at(j) = temp;
                    }
                }
            }
        }

        // put together close and far frontiers
        frontiers.clear();
        frontiers.reserve(close_frontiers.size() + far_frontiers.size());
        frontiers.insert(frontiers.end(), close_frontiers.begin(), close_frontiers.end());
        frontiers.insert(frontiers.end(), far_frontiers.begin(), far_frontiers.end());
    }
    
    release_mutex(&store_frontier_mutex, __FUNCTION__);

    ROS_INFO("Done sorting");
}

void ExplorationPlanner::sort_cost_reserve(bool energy_above_th, int w1, int w2, int w3, int w4)
{
//    tf::Stamped < tf::Pose > robotPose;
//    if(!costmap_ros_->getRobotPose(robotPose))
//    {
//        ROS_ERROR("Failed to get RobotPose");
//        return;
//    }

//    // process only eight frontiers
//    int max_front = 8;

//    if(sorted_frontiers.size() > 0)
//    {
//        //ROS_ERROR("%u", sorted_frontiers.size());
//        double robot_x, robot_y;
//        
//        //for(int i = sorted_frontiers.size(); i >= 0 && i > sorted_frontiers.size() - max_front; --i)
//        bool continue_bool = true;
//        for(int i = sorted_frontiers.size() - 1; continue_bool && i >= 0 && i > sorted_frontiers.size() - max_front - 1; --i)
//        {
//            //continue_bool = false;
//            //return;
//            //ROS_ERROR("sort: %d", (int)sorted_frontiers.size() - 1 - i);
//            if( ((int)sorted_frontiers.size() - 1 - i) < 0) {
//                ROS_FATAL("Somethign bad happened....");
//                return;
//            }
//            
//            //TEMP
//            //if((int)sorted_frontiers.size() - 1 - i >= 1)
//            //    return;
//                
//            for(int j = 0; j < sorted_frontiers.size()-1 && j < max_front; ++j)
//            {
//                //ROS_ERROR("sort2: %d", j);
//                /*
//                 * cost function
//                 * f = w1 · d_g   +   w2 · d_gb   +   w3 · d_gbe   +   w4 · theta
//                 *
//                 * parameters
//                 * w1, ..., w4 .. weights
//                 * d_g         .. distance from the robot's current position to the frontier
//                 * d_gb        .. distance from the frontier to the charging station
//                 * d_gbe       .. -d_gb if battery charge is above threshold (e.g. 50%), d_gb if battery charge is below threshold
//                 * theta       .. measure of how much the robot has to turn to reach frontier, theta in [0,1]
//                 */

//                // robot position
//                robot_x = robotPose.getOrigin().getX();
//                robot_y = robotPose.getOrigin().getY();

//                // frontier position
//                double frontier_x = sorted_frontiers.at(j).x_coordinate;
//                double frontier_y = sorted_frontiers.at(j).y_coordinate;
//                double next_frontier_x = sorted_frontiers.at(j+1).x_coordinate;
//                double next_frontier_y = sorted_frontiers.at(j+1).y_coordinate;

//                // calculate d_g
//                int d_g = trajectory_plan(frontier_x, frontier_y);
//                int d_g_next = trajectory_plan(next_frontier_x, next_frontier_y);

//                // calculate d_gb
//                int d_gb = trajectory_plan(frontier_x, frontier_y, robot_home_x, robot_home_y);
//                int d_gb_next = trajectory_plan(next_frontier_x, next_frontier_y, robot_home_x, robot_home_y);

//                // calculate d_gbe
//                int d_gbe, d_gbe_next;
//                if(energy_above_th)
//                {
//                    d_gbe = -d_gb;
//                    d_gbe_next = -d_gb_next;
//                }
//                else
//                {
//                    d_gbe = d_gb;
//                    d_gbe_next = d_gb_next;
//                }

//                // calculate theta
//                double theta_s = atan2(robot_last_y - robot_y, robot_last_x - robot_x);
//                double theta_g = atan2(robot_y - frontier_y, robot_x - frontier_x);
//                double theta_g_next = atan2(robot_y - next_frontier_y, robot_x - next_frontier_x);
//                double theta = 1/M_PI * (M_PI - abs(abs(theta_s - theta_g) - M_PI));
//                double theta_next = 1/M_PI * (M_PI - abs(abs(theta_s - theta_g_next) - M_PI));

//                // calculate cost function
//                double cost = w1 * d_g + w2 * d_gb + w3 * d_gbe + w4 * theta;
//                double cost_next = w1 * d_g_next + w2 * d_gb_next + w3 * d_gbe_next + w4 * theta_next;

//                // sort sorted_frontiers according to cost function
//                if(cost > cost_next)
//                {
//                    frontier_t temp = sorted_frontiers.at(j+1);
//                    sorted_frontiers.at(j+1) = sorted_frontiers.at(j);
//                    sorted_frontiers.at(j) = temp;
//                }
//            }
//        }
//        robot_last_x = robot_x;
//        robot_last_y = robot_y;
//    }
//    else
//    {
//        ROS_INFO("Sorting not possible, no frontiers available!!!");
//    }
//    
}

//void ExplorationPlanner::sort_cost_1(bool energy_above_th, int w1, int w2, int w3, int w4)
//{
//    //ROS_ERROR("calling %s", sc_distance_frontier_robot.getService().c_str());
//    //explorer::Distance distance_srv_msg;
//    //distance_srv_msg.request.x1 = 10;
//    //ros::service::waitForService("energy_mgmt/distance_on_graph");
//    //publish_frontier_list();
//    
//    /*
//    if(sc_distance_frontier_robot.call(distance_srv_msg))
//        ; //ROS_ERROR("call ok!");
//    else
//        ROS_ERROR("call failed!!!");
//    */
//    
//    /*
//    while(num_ds <= 0) {
//        ROS_ERROR("waiting DS count");
//        ros::Duration(2).sleep();
//        ros::spinOnce();
//    }
//    */
//    
//    if(num_ds <= 0)
//        return;
//    

//#ifndef QUICK_SELECTION

//    acquire_mutex(&store_frontier_mutex, __FUNCTION__);
//    
//    my_energy_above_th = energy_above_th;
//    this-> w1 = w1;
//    this-> w2 = w2;
//    this-> w3 = w3;
//    this-> w4 = w4;

//    // process only eight frontiers
//    int max_front = 8;

//    if(frontiers.size() > 0)
//    {
//        //ROS_ERROR("%u", frontiers.size());
//        double robot_x, robot_y;
//        
//        for(int i = frontiers.size()-1; i >= 0 && i > frontiers.size() - max_front; --i)
//        //for(int i = frontiers.size(); i >= 0 && i > frontiers.size() - max_front - skipped_due_to_auction; --i)
//        {
//            
//            //continue_bool = false;
//            //return;
//            //ROS_ERROR("sort: %d", (int)frontiers.size() - 1 - i);
//            if( ((int)frontiers.size() - 1 - i) < 0) {
//                ROS_FATAL("Somethign bad happened....");
//                release_mutex(&store_frontier_mutex, __FUNCTION__);
//                return;
//            }
//            
//            //TEMP
//            //if((int)frontiers.size() - 1 - i >= 1)
//            //    return;
//                
//            for(int j = 0; j < frontiers.size()-1 && j < max_front; ++j)
//            {

//                //ROS_ERROR("sort2: %d", j);
//                /*
//                 * cost function
//                 * f = w1 · d_g   +   w2 · d_gb   +   w3 · d_gbe   +   w4 · theta
//                 *
//                 * parameters
//                 * w1, ..., w4 .. weights
//                 * d_g         .. distance from the robot's current position to the frontier
//                 * d_gb        .. distance from the frontier to the charging station
//                 * d_gbe       .. -d_gb if battery charge is above threshold (e.g. 50%), d_gb if battery charge is below threshold
//                 * theta       .. measure of how much the robot has to turn to reach frontier, theta in [0,1]
//                 */

//                // robot position
//                robot_x = robotPose.getOrigin().getX();
//                robot_y = robotPose.getOrigin().getY();

//                // frontier position
//                double frontier_x = frontiers.at(j).x_coordinate;
//                double frontier_y = frontiers.at(j).y_coordinate;
//                double next_frontier_x = frontiers.at(j+1).x_coordinate;
//                double next_frontier_y = frontiers.at(j+1).y_coordinate;

//                // calculate d_g
//                int d_g = trajectory_plan_meters(frontier_x, frontier_y);
//                int d_g_next = trajectory_plan_meters(next_frontier_x, next_frontier_y);

//                // calculate d_gb
//                int d_gb = trajectory_plan_meters(frontier_x, frontier_y, robot_home_x, robot_home_y);
//                int d_gb_next = trajectory_plan_meters(next_frontier_x, next_frontier_y, robot_home_x, robot_home_y);

//                // calculate d_gbe
//                int d_gbe, d_gbe_next;
//                if(energy_above_th)
//                {
//                    d_gbe = -d_gb;
//                    d_gbe_next = -d_gb_next;
//                }
//                else
//                {
//                    d_gbe = d_gb;
//                    d_gbe_next = d_gb_next;
//                }

//                // calculate theta
//                double theta_s = atan2(robot_last_y - robot_y, robot_last_x - robot_x);
//                double theta_g = atan2(robot_y - frontier_y, robot_x - frontier_x);
//                double theta_g_next = atan2(robot_y - next_frontier_y, robot_x - next_frontier_x);
//                double theta = 1/M_PI * (M_PI - abs(abs(theta_s - theta_g) - M_PI));
//                double theta_next = 1/M_PI * (M_PI - abs(abs(theta_s - theta_g_next) - M_PI));

//                // calculate cost function
//                double cost = w1 * d_g + w2 * d_gb + w3 * d_gbe + w4 * theta;
//                double cost_next = w1 * d_g_next + w2 * d_gb_next + w3 * d_gbe_next + w4 * theta_next;

//                // sort frontiers according to cost function
//                //F ascending order
//                if(cost > cost_next)
//                {
//                    frontier_t temp = frontiers.at(j+1);
//                    frontiers.at(j+1) = frontiers.at(j);
//                    frontiers.at(j) = temp;
//                }
//            }
//        }
//        robot_last_x = robot_x;
//        robot_last_y = robot_y;
//    }
//    else
//    {
//        ROS_INFO("Sorting not possible, no frontiers available!!!");
//    }
//    
//    release_mutex(&store_frontier_mutex, __FUNCTION__);
//    ROS_INFO("finished sort cost");
//    
//#endif
// 
//}

/**
 * Sort frontiers in an energy aware manner according to a cost function with the weights w1, ..., w4
 */
//void ExplorationPlanner::sort_cost(bool energy_above_th, int w1, int w2, int w3, int w4)
//{

//#ifndef QUICK_SELECTION

//    //ROS_INFO("waiting for lock");
//    store_frontier_mutex.lock();
//    //ROS_INFO("lock acquired");
//    
//    tf::Stamped < tf::Pose > robotPose;
//    if(!costmap_ros_->getRobotPose(robotPose))
//    {
//        ROS_ERROR("Failed to get RobotPose");
//        release_mutex(&store_frontier_mutex, __FUNCTION__);
//        return;
//    }
//    
//    my_energy_above_th = energy_above_th;
//    this-> w1 = w1;
//    this-> w2 = w2;
//    this-> w3 = w3;
//    this-> w4 = w4;

//    // process only eight frontiers
//    int max_front = 8;

//    if(frontiers.size() > 0)
//    {
//        //ROS_ERROR("%u", frontiers.size());
//        double robot_x, robot_y;
//        
//        for(int i = frontiers.size()-1; i >= 0 && i > frontiers.size() - max_front; --i)
//        //for(int i = frontiers.size(); i >= 0 && i > frontiers.size() - max_front - skipped_due_to_auction; --i)
//        {
//          
//            //continue_bool = false;
//            //return;
//            //ROS_ERROR("sort: %d", (int)frontiers.size() - 1 - i);
//            if( ((int)frontiers.size() - 1 - i) < 0) {
//                ROS_FATAL("Somethign bad happened....");
//                release_mutex(&store_frontier_mutex, __FUNCTION__);
//                return;
//            }
//            
//            //TEMP
//            //if((int)frontiers.size() - 1 - i >= 1)
//            //    return;
//                
//            for(int j = 0; j < frontiers.size()-1 && j < max_front; ++j)
//            {

//                //ROS_ERROR("sort2: %d", j);
//                /*
//                 * cost function
//                 * f = w1 · d_g   +   w2 · d_gb   +   w3 · d_gbe   +   w4 · theta
//                 *
//                 * parameters
//                 * w1, ..., w4 .. weights
//                 * d_g         .. distance from the robot's current position to the frontier
//                 * d_gb        .. distance from the frontier to the charging station
//                 * d_gbe       .. -d_gb if battery charge is above threshold (e.g. 50%), d_gb if battery charge is below threshold
//                 * theta       .. measure of how much the robot has to turn to reach frontier, theta in [0,1]
//                 */

//                // robot position
//                robot_x = robotPose.getOrigin().getX();
//                robot_y = robotPose.getOrigin().getY();

//                // frontier position
//                double frontier_x = frontiers.at(j).x_coordinate;
//                double frontier_y = frontiers.at(j).y_coordinate;
//                double next_frontier_x = frontiers.at(j+1).x_coordinate;
//                double next_frontier_y = frontiers.at(j+1).y_coordinate;

//                // calculate d_g
//                int d_g = trajectory_plan(frontier_x, frontier_y);
//                int d_g_next = trajectory_plan(next_frontier_x, next_frontier_y);

//                // calculate d_gb
//                int d_gb = trajectory_plan(frontier_x, frontier_y, robot_home_x, robot_home_y);
//                int d_gb_next = trajectory_plan(next_frontier_x, next_frontier_y, robot_home_x, robot_home_y);

//                // calculate d_gbe
//                int d_gbe, d_gbe_next;
//                if(energy_above_th)
//                {
//                    d_gbe = -d_gb;
//                    d_gbe_next = -d_gb_next;
//                }
//                else
//                {
//                    d_gbe = d_gb;
//                    d_gbe_next = d_gb_next;
//                }

//                // calculate theta
//                double theta_s = atan2(robot_last_y - robot_y, robot_last_x - robot_x);
//                double theta_g = atan2(robot_y - frontier_y, robot_x - frontier_x);
//                double theta_g_next = atan2(robot_y - next_frontier_y, robot_x - next_frontier_x);
//                double theta = 1/M_PI * (M_PI - abs(abs(theta_s - theta_g) - M_PI));
//                double theta_next = 1/M_PI * (M_PI - abs(abs(theta_s - theta_g_next) - M_PI));

//                // calculate cost function
//                double cost = w1 * d_g + w2 * d_gb + w3 * d_gbe + w4 * theta;
//                double cost_next = w1 * d_g_next + w2 * d_gb_next + w3 * d_gbe_next + w4 * theta_next;

//                // sort frontiers according to cost function
//                //F ascending order
//                if(cost > cost_next)
//                {
//                    frontier_t temp = frontiers.at(j+1);
//                    frontiers.at(j+1) = frontiers.at(j);
//                    frontiers.at(j) = temp;
//                }
//            }
//        }
//        robot_last_x = robot_x;
//        robot_last_y = robot_y;
//    }
//    else
//    {
//        ROS_INFO("Sorting not possible, no frontiers available!!!");
//    }
//    
//    release_mutex(&store_frontier_mutex, __FUNCTION__);
//    ROS_INFO("finished sort cost");
//    
//#endif
// 
//}

void ExplorationPlanner::my_select_4(double available_distance, bool energy_above_th, int w1, int w2, int w3, int w4, std::vector<double> *final_goal, std::vector<std::string> *robot_str_name)
{
    
    // robot position
    double robot_x = robotPose.getOrigin().getX();
    double robot_y = robotPose.getOrigin().getY();
    
    my_energy_above_th = energy_above_th;
    this->w1 = w1;
    this->w2 = w2;
    this->w3 = w3;
    this->w4 = w4;
    
    if(frontiers.size() > 0)
    {
        ROS_INFO("%u", frontiers.size());
        
        double min_cost = std::numeric_limits<double>::max();    
        
        acquire_mutex(&store_frontier_mutex, __FUNCTION__);
        for(int j = 0; j < frontiers.size(); j++)
        {
        
            if(!my_check_efficiency_of_goal(available_distance, &frontiers.at(j)))
                continue;
                
            bool under_auction = false;
            for(int i=0; i < frontiers_under_auction.size(); i++)
                if(frontiers.at(j).id == frontiers_under_auction.at(i).id)
                    under_auction = true;
            if(under_auction)
                continue;
            
            frontiers.at(j).cost = frontier_cost(&frontiers.at(j));
            add_to_sorted_fontiers_list_if_convinient(frontiers.at(j));
            
            if(frontiers.at(j).cost < min_cost) {
                frontier_selected = true;
                min_cost = frontiers.at(j).cost;
            }
                  
        }
        release_mutex(&store_frontier_mutex, __FUNCTION__);
        
        robot_last_x = robot_x;
        robot_last_y = robot_y;
        
    }
    else
    {
        ROS_INFO("Sorting not possible, no frontiers available");
    }
    
    if(frontier_selected) {
        
        final_goal->push_back(my_selected_frontier->x_coordinate);
        final_goal->push_back(my_selected_frontier->y_coordinate);
        ROS_ERROR("%.2f, %.2f", my_selected_frontier->x_coordinate, my_selected_frontier->y_coordinate);
        final_goal->push_back(my_selected_frontier->detected_by_robot);
        final_goal->push_back(my_selected_frontier->id);
        
        robot_str_name->push_back(robot_name_str);  //TODO ???
    
    }
    
    release_mutex(&store_frontier_mutex, __FUNCTION__);
    ROS_INFO("finished my_select_4");
  
}

//void ExplorationPlanner::my_sort_cost_3(bool energy_above_th, int w1, int w2, int w3, int w4)
//{
//    
//    if(ds_list.size() == 0) {
//        ROS_INFO("No DS is currently known: falling back to closest st");
//        my_sort_cost_2(energy_above_th, w1, w2, w3, w4);
//        return;   
//    }
//    
//    ROS_INFO("Using this strategy");
//    
//    // robot position
//    double robot_x = robotPose.getOrigin().getX();
//    double robot_y = robotPose.getOrigin().getY();

////    if(recompute_ds_graph) {
////        recompute_ds_graph = false;
////        for(int i=0; i < ds_list.size(); i++)
////            for(int j=0; j < ds_list.size(); j++)
////                if(ds_graph[i][j] >= 0 || i == j)
////                    continue;
////                else {
////                    double dist = trajectory_plan(ds_list[i].x, ds_list[i].y, ds_list[j].x, ds_list[j].y);
////                    if(dist < 0) {
////                        ROS_ERROR("Unable to compute distance at the moment: retrying later...");
////                        recompute_ds_graph = true;
////                    }
////                    else {
////                        ds_graph[ds_list[i].id][ds_list[j].id] = dist;
////                        ds_graph[ds_list[j].id][ds_list[i].id] = dist;
////                    }
////                }
////    }
//    
//    acquire_mutex(&store_frontier_mutex, __FUNCTION__);
//    
//    int index_closest_ds_to_robot = -1, index_optimal_ds = -1;
//    double min_dist = std::numeric_limits<double>::max();
//    for(unsigned int i=0; i < ds_list.size(); i++) {
//        double distance = euclidean_distance(robot_x, robot_y, ds_list.at(i).x, ds_list.at(i).y);
//        if(distance < min_dist) {
//            min_dist = distance;
//            index_closest_ds_to_robot = i;
//        }
//        if(ds_list.at(i).id == optimal_ds_id)
//            index_optimal_ds = i;
//    }
//    
//    for(unsigned int i=0; i < frontiers.size(); i++) {
//        double min_dist = std::numeric_limits<double>::max();
//        int index_closest_ds_to_frontier = -1;
//        for(unsigned int j=0; j < ds_list.size(); j++) {
//            double distance = euclidean_distance(frontiers.at(i).x_coordinate, frontiers.at(i).y_coordinate, ds_list.at(j).x, ds_list.at(j).y);
//            if(distance < min_dist) {
//                min_dist = distance;
//                index_closest_ds_to_frontier = j;
//            }
//        }
//        
//        if(index_optimal_ds >= ds_graph.size() || index_closest_ds_to_robot >= ds_graph[index_optimal_ds].size() || index_closest_ds_to_robot < 0 || index_optimal_ds < 0 || index_closest_ds_to_frontier < 0)
//            ROS_ERROR("invalid index(-ces)"); //: %d, %d, %u", index_optimal_ds, index_closest_ds_to_robot, ds_graph.size());
//            //ROS_ERROR("index_optimal_ds is too high; ds_graph size: %u; index_optimal_ds: %d", ds_graph.size(), index_optimal_ds);
//        
//        //double d_gbe = dijkstra(ds_graph, index_optimal_ds, index_closest_ds_to_robot);
//        double d_gbe = ds_graph[index_optimal_ds][index_closest_ds_to_robot];
//        if(d_gbe < 0)
//            ROS_ERROR("Invalid value");
//        //double d_g = dijkstra(ds_graph, index_closest_ds_to_frontier, index_closest_ds_to_robot);
//        double d_g = ds_graph[index_closest_ds_to_frontier][index_closest_ds_to_robot];
//        if(d_g < 0)
//            ROS_ERROR("Invalid value");
//        double cost = d_g + d_gbe;
//        frontiers.at(i).cost = cost;
//        
//        add_to_sorted_fontiers_list_if_convinient(frontiers.at(i));
//        
//    }
//    
//    frontier_selected = true;
//    my_energy_above_th = energy_above_th;
//    this->w1 = w1;
//    this->w2 = w2;
//    this->w3 = w3;
//    this->w4 = w4;
//    
//    robot_last_x = robot_x;
//    robot_last_y = robot_y;
//    
//    ROS_INFO("finished my_sort_cost_2");
//    release_mutex(&store_frontier_mutex, __FUNCTION__);
//  
//}

void ExplorationPlanner::my_sort_cost_2(bool energy_above_th, int w1, int w2, int w3, int w4)
{
    acquire_mutex(&store_frontier_mutex, __FUNCTION__);
    
    // robot position
    double robot_x = robotPose.getOrigin().getX();
    double robot_y = robotPose.getOrigin().getY();
    
    this->my_energy_above_th = energy_above_th;
    this->w1 = w1;
    this->w2 = w2;
    this->w3 = w3;
    this->w4 = w4;

    for (int j = 0; j < frontiers.size() - 1; j++) {
        double x = frontiers.at(j).x_coordinate - robotPose.getOrigin().getX();
        double y = frontiers.at(j).y_coordinate - robotPose.getOrigin().getY();
        double cost = x * x + y * y;
        frontiers.at(j).cost = cost;
        
        add_to_sorted_fontiers_list_if_convinient(frontiers.at(j));
        
    }
    
    frontier_selected = true;
    ROS_DEBUG("sorted_frontiers size: %u", sorted_frontiers.size());
        
    robot_last_x = robot_x;
    robot_last_y = robot_y;
    
    release_mutex(&store_frontier_mutex, __FUNCTION__);
    ROS_INFO("finished my_sort_cost_2");
  
}

//void ExplorationPlanner::smart_sort_cost(bool energy_above_th, int w1, int w2, int w3, int w4)
//{

//    //ROS_INFO("waiting for lock");
//    //store_frontier_mutex.lock();
//    acquire_mutex(&store_frontier_mutex, __FUNCTION__);
//    //ROS_INFO("lock acquired");
//    
//    tf::Stamped < tf::Pose > robotPose;
//    if(!costmap_ros_->getRobotPose(robotPose))
//    {
//        ROS_ERROR("Failed to get RobotPose");
//        release_mutex(&store_frontier_mutex, __FUNCTION__);
//        return;
//    }
//    
//    my_energy_above_th = energy_above_th;
//    this-> w1 = w1;
//    this-> w2 = w2;
//    this-> w3 = w3;
//    this-> w4 = w4;

//    // process only eight frontiers
//    //int max_front = 8;
//    int max_front = frontiers.size();
//    sorted_frontiers.clear();
//    
//    if(frontiers.size() > 0)
//    {
//        //ROS_ERROR("%u", frontiers.size());
//        double robot_x, robot_y;

//        //continue_bool = false;
//        //return;
//        //ROS_ERROR("sort: %d", (int)frontiers.size() - 1 - i);
//        //if( ((int)frontiers.size() - 1 - i) < 0) {
//        //    ROS_FATAL("Somethign bad happened....");
//        //    release_mutex(&store_frontier_mutex, __FUNCTION__);
//        //    return;
//        //}
//        
//        //TEMP
//        //if((int)frontiers.size() - 1 - i >= 1)
//        //    return;
//            
//        for(int j = 0; j < frontiers.size()-1 && j < max_front; ++j)
//        {

//            //ROS_ERROR("sort2: %d", j);
//            /*
//             * cost function
//             * f = w1 · d_g   +   w2 · d_gb   +   w3 · d_gbe   +   w4 · theta
//             *
//             * parameters
//             * w1, ..., w4 .. weights
//             * d_g         .. distance from the robot's current position to the frontier
//             * d_gb        .. distance from the frontier to the charging station
//             * d_gbe       .. -d_gb if battery charge is above threshold (e.g. 50%), d_gb if battery charge is below threshold
//             * theta       .. measure of how much the robot has to turn to reach frontier, theta in [0,1]
//             */

//            // robot position
//            robot_x = robotPose.getOrigin().getX();
//            robot_y = robotPose.getOrigin().getY();

//            // frontier position
//            double frontier_x = frontiers.at(j).x_coordinate;
//            double frontier_y = frontiers.at(j).y_coordinate;
//            double next_frontier_x = frontiers.at(j+1).x_coordinate;
//            double next_frontier_y = frontiers.at(j+1).y_coordinate;

//            // calculate d_g
//            int d_g = trajectory_plan(frontier_x, frontier_y);
//            int d_g_next = trajectory_plan(next_frontier_x, next_frontier_y);

//            // calculate d_gb
//            int d_gb = trajectory_plan(frontier_x, frontier_y, robot_home_x, robot_home_y);
//            int d_gb_next = trajectory_plan(next_frontier_x, next_frontier_y, robot_home_x, robot_home_y);

//            // calculate d_gbe
//            int d_gbe, d_gbe_next;
//            if(energy_above_th)
//            {
//                d_gbe = -d_gb;
//                d_gbe_next = -d_gb_next;
//            }
//            else
//            {
//                d_gbe = d_gb;
//                d_gbe_next = d_gb_next;
//            }

//            // calculate theta
//            double theta_s = atan2(robot_last_y - robot_y, robot_last_x - robot_x);
//            double theta_g = atan2(robot_y - frontier_y, robot_x - frontier_x);
//            double theta_g_next = atan2(robot_y - next_frontier_y, robot_x - next_frontier_x);
//            double theta = 1/M_PI * (M_PI - abs(abs(theta_s - theta_g) - M_PI));
//            double theta_next = 1/M_PI * (M_PI - abs(abs(theta_s - theta_g_next) - M_PI));

//            // calculate cost function
//            double cost = w1 * d_g + w2 * d_gb + w3 * d_gbe + w4 * theta;
//            double cost_next = w1 * d_g_next + w2 * d_gb_next + w3 * d_gbe_next + w4 * theta_next;

//            // sort frontiers according to cost function
//            //F ascending order
//            if(cost > cost_next)
//            {
//                frontier_t temp = frontiers.at(j+1);
//                frontiers.at(j+1) = frontiers.at(j);
//                frontiers.at(j) = temp;
//            }
//            
//            //bool to_be_inserted
//            //if(sorted_frontiers.size() == 0)
//            //    sorted_frontier.push_back(frontiers.at(j));
//            //else
//            //    for(int k=0; k < sorted_frontiers.size(); k++)
//            //        if(   
//            
//        }
//        
//        robot_last_x = robot_x;
//        robot_last_y = robot_y;
//    }
//    else
//    {
//        ROS_INFO("Sorting not possible, no frontiers available!!!");
//    }
//    
//    release_mutex(&store_frontier_mutex, __FUNCTION__);
//    ROS_INFO("finished sort cost");
//  
//}

bool ExplorationPlanner::determine_goal(int strategy, std::vector<double> *final_goal, int count, int actual_cluster_id, std::vector<std::string> *robot_str_name)
{
    if (!costmap_ros_->getRobotPose(robotPose))
    {
            ROS_ERROR("Failed to get RobotPose");
    }

    if(strategy == 1)
    {
            for (int i = frontiers.size() - 1 - count; i >= 0; i--)
            {
                    if (check_efficiency_of_goal(frontiers.at(i).x_coordinate, frontiers.at(i).y_coordinate) == true)
                    {
                            ROS_INFO("------------------------------------------------------------------");
                            ROS_INFO("Determined frontier with ID: %d   at x: %f     y: %f   detected by Robot %d", frontiers.at(i).id, frontiers.at(i).x_coordinate, frontiers.at(i).y_coordinate, frontiers.at(i).detected_by_robot);

                            /*
                             * Solve the Problem of planning a path to a frontier which is located
                             * directly "in" the obstacle. Therefore back-off 5% of the targets
                             * coordinate. Since the direction of x and y can be positive and
                             * negative either, multiply the coordinate with 0.95 to get 95% of its
                             * original value.
                             */
                            final_goal->push_back(frontiers.at(i).x_coordinate); //final_goal.push_back(frontiers.at(i).x_coordinate*0.95 - robotPose.getOrigin().getX());
                            final_goal->push_back(frontiers.at(i).y_coordinate);
                            final_goal->push_back(frontiers.at(i).detected_by_robot);
                            final_goal->push_back(frontiers.at(i).id);

                            robot_str_name->push_back(frontiers.at(i).detected_by_robot_str);
                            return true;
                    }
            }
            return false;
        }

        else if(strategy == 2)
        {
            for (unsigned int i = 0 + count; i < frontiers.size(); i++)
            {
                if (check_efficiency_of_goal(frontiers.at(i).x_coordinate, frontiers.at(i).y_coordinate) == true)
                {
                        ROS_INFO("------------------------------------------------------------------");
                        ROS_INFO("Determined frontier with ID: %d   at x: %f     y: %f   detected by Robot %d", frontiers.at(i).id, frontiers.at(i).x_coordinate, frontiers.at(i).y_coordinate, frontiers.at(i).detected_by_robot);

                        final_goal->push_back(frontiers.at(i).x_coordinate);
                        final_goal->push_back(frontiers.at(i).y_coordinate);
                        final_goal->push_back(frontiers.at(i).detected_by_robot);
                        final_goal->push_back(frontiers.at(i).id);

                        robot_str_name->push_back(frontiers.at(i).detected_by_robot_str);
                        return true;
                }
            }
        }

        else if(strategy == 3)
        {
            while(true)
            {
                if(frontiers.size() > 0)
                {

                    int i = int(frontiers.size()*rand()/(RAND_MAX));

                    ROS_INFO("Random frontier ID: %d", frontiers.at(i).id);

                    if (check_efficiency_of_goal(frontiers.at(i).x_coordinate, frontiers.at(i).y_coordinate) == true)
                    {
                            ROS_INFO("------------------------------------------------------------------");
                            ROS_INFO("Determined frontier with ID: %d   at x: %f     y: %f   detected by Robot %d", frontiers.at(i).id, frontiers.at(i).x_coordinate, frontiers.at(i).y_coordinate, frontiers.at(i).detected_by_robot);

                            final_goal->push_back(frontiers.at(i).x_coordinate);
                            final_goal->push_back(frontiers.at(i).y_coordinate);
                            final_goal->push_back(frontiers.at(i).detected_by_robot);
                            final_goal->push_back(frontiers.at(i).id);

                            robot_str_name->push_back(frontiers.at(i).detected_by_robot_str);
                            return true;
                    }
                }
                break;
            }
	}
        else if(strategy == 4)
        {
                int cluster_vector_position = 0;

                if(actual_cluster_id != -1)
                {
                    if(clusters.size() > 0)
                    {
                        for (unsigned int i = 0; i < clusters.size(); i++)
                        {
                            if(clusters.at(i).id == actual_cluster_id)
                            {
                                if(clusters.at(i).cluster_element.size() > 0)
                                {
                                    cluster_vector_position = i;
                                }
                                break;
                            }
                        }
                    }
                }

                ROS_INFO("Calculated vector position of cluster %d", actual_cluster_id);
                /*
                 * Iterate over all clusters .... if cluster_vector_position is set
                 * also the clusters with a lower have to be checked if the frontier
                 * determination fails at clusters.at(cluster_vector_position). therefore
                 * a ring-buffer is operated to iterate also over lower clusters, since
                 * they might have changed.
                 */
                int nothing_found_in_actual_cluster = 0;
                int visited_clusters = 0;
                for (unsigned int i = 0 + count; i < clusters.size(); i++)
                {
//                    ROS_INFO("Cluster vector: %d  i: %d ", cluster_vector_position, i);
                    i = i+ cluster_vector_position;
                    i = i % (clusters.size());
                    for (unsigned int j = 0; j < clusters.at(i).cluster_element.size(); j++)
                    {
                        if (check_efficiency_of_goal(clusters.at(i).cluster_element.at(j).x_coordinate, clusters.at(i).cluster_element.at(j).y_coordinate) == true)
                        {
                                ROS_INFO("------------------------------------------------------------------");
                                ROS_INFO("Robot %d determined Goal: %d  at Clusters: %d",robot_name, (int)clusters.at(i).cluster_element.at(j).id, (int)clusters.at(i).id);

                                final_goal->push_back(clusters.at(i).cluster_element.at(j).x_coordinate);
                                final_goal->push_back(clusters.at(i).cluster_element.at(j).y_coordinate);
                                final_goal->push_back(clusters.at(i).cluster_element.at(j).detected_by_robot);
                                final_goal->push_back(clusters.at(i).cluster_element.at(j).id);

                                // number of the cluster we operate in
                                final_goal->push_back(clusters.at(i).id);
                                robot_str_name->push_back(clusters.at(i).cluster_element.at(j).detected_by_robot_str);
                                return true;
                        }

                    }

                    nothing_found_in_actual_cluster ++;
                    visited_clusters ++;

                    if(nothing_found_in_actual_cluster == 1)
                    {
                        //start again at the beginning(closest cluster))
                        i=0;
                        cluster_vector_position = 0;
                    }

                    if(visited_clusters == clusters.size())
                    {
                        break;
                    }
               }
            }
            else if(strategy == 5)
            {
                int cluster_vector_position = 0;
                bool cluster_could_be_found = false;

                if(actual_cluster_id != -1)
                {
                    if(clusters.size() > 0)
                    {
                        for (unsigned int i = 0; i < clusters.size(); i++)
                        {
                            if(clusters.at(i).id == actual_cluster_id)
                            {
                                if(clusters.at(i).cluster_element.size() > 0)
                                {
                                    cluster_vector_position = i;
                                    cluster_could_be_found = true;
                                    break;
                                }
                            }
                        }
                    }
                    if(cluster_could_be_found == false)
                    {
                        /*
                         * The cluster we operated in is now empty
                         */
                        return false;
                    }
                }
                else if(actual_cluster_id == -1)
                {
                    // No cluster was previously selected
                    final_goal->push_back(-1.0);
                    return false;
                }
                ROS_INFO("Calculated vector position: %d of cluster %d", cluster_vector_position, actual_cluster_id);
                /*
                 * Iterate over all clusters .... if cluster_vector_position is set
                 * also the clusters with a lower have to be checked if the frontier
                 * determination fails at clusters.at(cluster_vector_position). therefore
                 * a ring-buffer is operated to iterate also over lower clusters, since
                 * they might have changed.
                 */
                int nothing_found_in_actual_cluster = 0;
                int visited_clusters = 0;

//                if(clusters.size() > 0)
//                {

                    int position = (cluster_vector_position +count) % (clusters.size());
                    for (unsigned int j = 0; j < clusters.at(position).cluster_element.size(); j++)
                    {
                        if(count >= clusters.size())
                        {
                            break;
                        }

                        if(clusters.at(position).cluster_element.size() > 0)
                        {
                            if (check_efficiency_of_goal(clusters.at(position).cluster_element.at(j).x_coordinate, clusters.at(position).cluster_element.at(j).y_coordinate) == true)
                            {
                                    ROS_INFO("------------------------------------------------------------------");
                                    ROS_INFO("Robot %d determined Goal: %d  at Clusters: %d",robot_name, (int)clusters.at(position).cluster_element.at(j).id, (int)clusters.at(position).id);

                                    final_goal->push_back(clusters.at(position).cluster_element.at(j).x_coordinate);
                                    final_goal->push_back(clusters.at(position).cluster_element.at(j).y_coordinate);
                                    final_goal->push_back(clusters.at(position).cluster_element.at(j).detected_by_robot);
                                    final_goal->push_back(clusters.at(position).cluster_element.at(j).id);

                                    // number of the cluster we operate in
                                    final_goal->push_back(clusters.at(position).id);
                                    robot_str_name->push_back(clusters.at(position).cluster_element.at(j).detected_by_robot_str);
                                    return true;
                            }
                        }

                    }

//                }
//                ROS_INFO("No clusters are available anymore");

                return false;
            }
            else if(strategy == 6)
            {
                int cluster_vector_position = 0;
                bool cluster_could_be_found = false;

                if(actual_cluster_id != -1)
                {
                    if(clusters.size() > 0)
                    {
                        for (unsigned int i = 0; i < clusters.size(); i++)
                        {
                            if(clusters.at(i).id == actual_cluster_id)
                            {
                                if(clusters.at(i).cluster_element.size() > 0)
                                {
                                    cluster_vector_position = i;
                                    cluster_could_be_found = true;
                                    break;
                                }
                            }
                        }
                    }
                    if(cluster_could_be_found == false)
                    {
                        /*
                         * The cluster we operated in is now empty
                         */
                        return false;
                    }
                }
                else if(actual_cluster_id == -1)
                {
                    // No cluster was previously selected
                    final_goal->push_back(-1.0);
                    return false;
                }
                ROS_INFO("Calculated vector position: %d of cluster %d", cluster_vector_position, actual_cluster_id);
                /*
                 * Iterate over all clusters .... if cluster_vector_position is set
                 * also the clusters with a lower have to be checked if the frontier
                 * determination fails at clusters.at(cluster_vector_position). therefore
                 * a ring-buffer is operated to iterate also over lower clusters, since
                 * they might have changed.
                 */
                int nothing_found_in_actual_cluster = 0;
                int visited_clusters = 0;

//                    ROS_INFO("position: %u", (cluster_vector_position +count) % (clusters.size()));
                    int position = (cluster_vector_position +count) % (clusters.size());
                    for (unsigned int j = 0; j < clusters.at(position).cluster_element.size(); j++)
                    {
                        if(count >= clusters.size())
                        {
                            break;
                        }

                        if(clusters.at(position).cluster_element.size() > 0)
                        {
                            if (check_efficiency_of_goal(clusters.at(position).cluster_element.at(j).x_coordinate, clusters.at(position).cluster_element.at(j).y_coordinate) == true)
                            {
                                    ROS_INFO("------------------------------------------------------------------");
                                    ROS_INFO("Robot %d determined Goal: %d  at Clusters: %d",robot_name, (int)clusters.at(position).cluster_element.at(j).id, (int)clusters.at(position).id);

                                    final_goal->push_back(clusters.at(position).cluster_element.at(j).x_coordinate);
                                    final_goal->push_back(clusters.at(position).cluster_element.at(j).y_coordinate);
                                    final_goal->push_back(clusters.at(position).cluster_element.at(j).detected_by_robot);
                                    final_goal->push_back(clusters.at(position).cluster_element.at(j).id);

                                    // number of the cluster we operate in
                                    final_goal->push_back(clusters.at(position).id);
                                    robot_str_name->push_back(clusters.at(position).cluster_element.at(j).detected_by_robot_str);
                                    return true;
                            }
//                            else
//                            {
//                                final_goal->push_back(-1);
//                                return true;
//                            }
                        }

                    }
                return false;
            }
     return false;
}

//bool ExplorationPlanner::determine_goal_staying_alive_2_reserve(int mode, int strategy, double available_distance, std::vector<double> *final_goal, int count, std::vector<std::string> *robot_str_name, int actual_cluster_id)
//{
//    if (!costmap_ros_->getRobotPose(robotPose))
//    {
//            ROS_ERROR("Failed to get RobotPose");
//    }

//    // check if robot needs to go home right away
//    double dist_home;
//    double dist_front;
//    double closest = 9999;
//    if(strategy == 1){
//        double xh = robot_home_x - robotPose.getOrigin().getX();
//        double yh = robot_home_y - robotPose.getOrigin().getY();
//        dist_home = sqrt(xh * xh + yh * yh) * 1.2; // 1.2 is extra reserve, just in case
//    }
//    else if(strategy == 2){
//        dist_home = trajectory_plan(robot_home_x, robot_home_y) * costmap_ros_->getCostmap()->getResolution();
//    }
//    if(dist_home > 0 && dist_home >= available_distance)
//        return false;

//    // look for a FRONTIER as goal
//    int errors = 0;
//    if (mode == 1)
//    {
//        for (int i = 0 + count; i < sorted_frontiers.size(); i++)
//        {
//            //we only check the first 9 frontiers, because we only sorted the first 9 frontiers by efficiency.
//            if(i>8){
//                // if the distances could not be computed, try again using euclidean distances instead
//                if(errors == i && strategy == 2){
//                    ROS_ERROR("Fallback to euclidean distance.");
//                    return this->determine_goal_staying_alive(1, 1, available_distance, final_goal, count, robot_str_name, -1);
//                }
//                return false;
//            }

//            if (check_efficiency_of_goal(sorted_frontiers.at(i).x_coordinate, sorted_frontiers.at(i).y_coordinate) == true)
//            {
//                double distance;
//                double total_distance;
//                if(strategy == 1){
//                    // distance to next frontier
//                    double x1 = sorted_frontiers.at(i).x_coordinate - robotPose.getOrigin().getX();
//                    double y1 = sorted_frontiers.at(i).y_coordinate -  robotPose.getOrigin().getY();
//                    // distance from frontier to home base
//                    double x2 = robot_home_x - sorted_frontiers.at(i).x_coordinate;
//                    double y2 = robot_home_y -  sorted_frontiers.at(i).y_coordinate;
//                    total_distance = sqrt(x1 * x1 + y1 * y1) + sqrt(x2 * x2 + y2 * y2);
//                }
//                else if(strategy == 2){
//                    // distance to next frontier
//                    total_distance = trajectory_plan(sorted_frontiers.at(i).x_coordinate, sorted_frontiers.at(i).y_coordinate);
//                    if(total_distance < 0){
//                        ROS_ERROR("Failed to compute distance!");
//                        errors++;
//                        continue;
//                    }
//                    // distance from frontier to home base
//                    distance = trajectory_plan(sorted_frontiers.at(i).x_coordinate, sorted_frontiers.at(i).y_coordinate, robot_home_x, robot_home_y);
//                    if(distance < 0){
//                        ROS_ERROR("Failed to compute distance!");
//                        errors++;
//                        continue;
//                    }
//                    total_distance += distance;

//                    if(total_distance < closest){
//                        closest = total_distance;
//                        dist_front = total_distance - distance;
//                        dist_home = distance;
//                    }

//                    // convert from cells to meters
//                    total_distance *= costmap_ros_->getCostmap()->getResolution();
//                }
//                else{
//                    ROS_ERROR("Wrong strategy, cannot compute distance to goal!");
//                    return false;
//                }

//                ROS_INFO("Distance to frontier and then home: %.2f",total_distance);
//                if(available_distance > total_distance)
//                {
//                    ROS_INFO("------------------------------------------------------------------");
//                    ROS_INFO("Determined frontier with ID: %d   at x: %.2f     y: %.2f   detected by Robot %d", sorted_frontiers.at(i).id, sorted_frontiers.at(i).x_coordinate, sorted_frontiers.at(i).y_coordinate, sorted_frontiers.at(i).detected_by_robot);

//                    final_goal->push_back(sorted_frontiers.at(i).x_coordinate);
//                    final_goal->push_back(sorted_frontiers.at(i).y_coordinate);
//                    final_goal->push_back(sorted_frontiers.at(i).detected_by_robot);
//                    final_goal->push_back(sorted_frontiers.at(i).id);

//                    robot_str_name->push_back(sorted_frontiers.at(i).detected_by_robot_str);
//                    return true;
//                    
//                }else{
//                    ROS_INFO("No frontier in energetic range (%.2f < %.2f + %.2f)", available_distance, dist_front * costmap_ros_->getCostmap()->getResolution(), dist_home * costmap_ros_->getCostmap()->getResolution());
//                    return false;
//                }
//            }
//        }
//    }

//    // look for a CLUSTER as goal
//    else if (mode == 2)
//    {
//        int cluster_vector_position = 0;

//        if(clusters.size() > 0)
//        {
//            for (int i = 0; i < clusters.size(); i++)
//            {
//                if(clusters.at(i).id == actual_cluster_id)
//                {
//                    if(clusters.at(i).cluster_element.size() > 0)
//                    {
//                        cluster_vector_position = i;
//                    }
//                    break;
//                }
//            }
//        }

//        ROS_INFO("Calculated vector position of cluster %d", actual_cluster_id);
//        /*
//         * Iterate over all clusters .... if cluster_vector_position is set
//         * also the clusters with a lower have to be checked if the frontier
//         * determination fails at clusters.at(cluster_vector_position). therefore
//         * a ring-buffer is operated to iterate also over lower clusters, since
//         * they might have changed.
//         */
//        int nothing_found_in_actual_cluster = 0;
//        int visited_clusters = 0;
//        for (int i = 0 + count; i < clusters.size(); i++)
//        {
//            i = i+ cluster_vector_position;
//            i = i % (clusters.size());
//            for (int j = 0; j < clusters.at(i).cluster_element.size(); j++)
//            {
//                if (check_efficiency_of_goal(clusters.at(i).cluster_element.at(j).x_coordinate, clusters.at(i).cluster_element.at(j).y_coordinate) == true)
//                {
//                    double distance;
//                    double total_distance;
//                    if(strategy == 1){
//                        // distance to cluster
//                        double x1 = clusters.at(i).cluster_element.at(j).x_coordinate - robotPose.getOrigin().getX();
//                        double y1 = clusters.at(i).cluster_element.at(j).y_coordinate - robotPose.getOrigin().getY();
//                        // distance from cluster to home base
//                        double x2 = robot_home_x - clusters.at(i).cluster_element.at(j).x_coordinate;
//                        double y2 = robot_home_y -  clusters.at(i).cluster_element.at(j).y_coordinate;
//                        total_distance = sqrt(x1 * x1 + y1 * y1) + sqrt(x2 * x2 + y2 * y2);
//                    }
//                    else if(strategy == 2){
//                        // distance to cluster
//                        total_distance = trajectory_plan(clusters.at(i).cluster_element.at(j).x_coordinate, clusters.at(i).cluster_element.at(j).y_coordinate);
//                        if(total_distance < 0){
//                            ROS_ERROR("Failed to compute distance!");
//                            errors++;
//                            continue;
//                        }
//                        // distance from cluster to home base
//                        distance = trajectory_plan(clusters.at(i).cluster_element.at(j).x_coordinate, clusters.at(i).cluster_element.at(j).y_coordinate, robot_home_x, robot_home_y);
//                        if(distance < 0){
//                            ROS_ERROR("Failed to compute distance!");
//                            errors++;
//                            continue;
//                        }
//                        total_distance += distance;
//                        // convert from cells to meters
//                        total_distance *= costmap_ros_->getCostmap()->getResolution();
//                    }
//                    else{
//                        ROS_ERROR("Wrong strategy, cannot compute distance to goal!");
//                        return false;
//                    }

//                    ROS_INFO("distance to cluster and then home: %f",total_distance);
//                    if(available_distance > total_distance)
//                    {
//                        ROS_INFO("------------------------------------------------------------------");
//                        ROS_INFO("Robot %d determined Goal: %d  at Clusters: %d",robot_name, (int)clusters.at(i).cluster_element.at(j).id, (int)clusters.at(i).id);

//                        final_goal->push_back(clusters.at(i).cluster_element.at(j).x_coordinate);
//                        final_goal->push_back(clusters.at(i).cluster_element.at(j).y_coordinate);
//                        final_goal->push_back(clusters.at(i).cluster_element.at(j).detected_by_robot);
//                        final_goal->push_back(clusters.at(i).cluster_element.at(j).id);

//                        // number of the cluster we operate in
//                        final_goal->push_back(clusters.at(i).id);
//                        robot_str_name->push_back(clusters.at(i).cluster_element.at(j).detected_by_robot_str);
//                        return true;
//                    }
//                }
//            }

//            nothing_found_in_actual_cluster ++;
//            visited_clusters ++;

//            if(nothing_found_in_actual_cluster == 1)
//            {
//                //start again at the beginning(closest cluster))
//                i=0;
//                cluster_vector_position = 0;
//            }

//            if(visited_clusters == clusters.size())
//            {
//                ROS_ERROR("No frontier in energetic range %.2f, going home for recharging", available_distance);
//                return false;
//            }
//        }
//    }
//    
//}

//void ExplorationPlanner::sort_cost_with_approach(bool energy_above_th, int w1, int w2, int w3, int w4)
//{
//    if(recompute_ds_graph) {
//        recompute_ds_graph = false;
//        for(int i=0; i < ds_list.size(); i++)
//            for(int j=0; j < ds_list.size(); j++)
//                if(ds_graph[i][j] >= 0 || i == j)
//                    continue;
//                else {
//                    double dist = trajectory_plan_meters(ds_list[i].x, ds_list[i].y, ds_list[j].x, ds_list[j].y);
//                    //ROS_ERROR("distance between (%.1f, %.1f) and (%.1f, %.1f): %f", ds_list[i].x, ds_list[i].y, ds_list[j].x, ds_list[j].y, dist);
//                    if(dist < 0) {
//                        ROS_ERROR("Unable to compute distance at the moment: retrying later...");
//                        recompute_ds_graph = true;
//                    }
//                    else {
//                        ds_graph[ds_list[i].id][ds_list[j].id] = dist;
//                        ds_graph[ds_list[j].id][ds_list[i].id] = dist;
//                    }
//                }
//    }
//    if(APPROACH == 0)
//        /* Original cost function */
//        sort_cost(energy_above_th, w1, w2, w3, w4);
//    else if(APPROACH == 1)
//        /* Cost function: (real distance = Disjktra's distance)
//         *     - (real) distance between the given frontier and target DS of the robot;
//         *     - distance on the graph of the DSs between the DS that is closest, according to the euclidean distance (not the real one!), to the given frontier and the robot; this distance is computed efficiently by the energy_mgmt node;
//         *     - d_r
//         *     - theta_rel
//         */
//        sort_cost_1(energy_above_th, w1, w2, w3, w4);
//    else if(APPROACH == -1)
//        return;
//    else
//        ROS_ERROR("INVALID APPROACH!!!");
//}

//bool ExplorationPlanner::determine_goal_staying_alive_2(int mode, int strategy, double available_distance, std::vector<double> *final_goal, int count, std::vector<std::string> *robot_str_name, int actual_cluster_id)
//{
//    if (!costmap_ros_->getRobotPose(robotPose))
//    {
//            ROS_ERROR("Failed to get RobotPose");
//    }
//    
//    //store_frontier_mutex.lock();
//    acquire_mutex(&store_frontier_mutex, __FUNCTION__);

//    // check if robot needs to go recharging right away
//    double dist_home;
//    double dist_front;
//    double closest = 9999;
//    if(strategy == 1){
//        double xh = optimal_ds_x - robotPose.getOrigin().getX();
//        double yh = optimal_ds_y - robotPose.getOrigin().getY();
//        dist_home = sqrt(xh * xh + yh * yh) * 1.2; // 1.2 is extra reserve, just in case
//    }
//    else if(strategy == 2){
//        dist_home = trajectory_plan(optimal_ds_x, optimal_ds_y) * costmap_ros_->getCostmap()->getResolution();
//    }
//    //ROS_ERROR("available_distance: %f", available_distance);
//    if(dist_home > 0 && dist_home >= available_distance) {
//        ROS_ERROR("Target DS is too far to reach a frontier...\noptimal_ds_x: %f, optimal_ds_y: %f, distance: %f, available distance: %f", optimal_ds_x, optimal_ds_y, dist_home, available_distance);
//        release_mutex(&store_frontier_mutex, __FUNCTION__);
//        return false;
//    }

//    // look for a FRONTIER as goal
//    int errors = 0;
//    if (mode == 1)
//    {
//        for (int i = 0 + count; i < frontiers.size(); i++)
//        {
//            //we only check the first 9 frontiers, because we only sorted the first 9 frontiers by efficiency.
//            if(i>8){
//                // if the distances could not be computed, try again using euclidean distances instead
//                if(errors == i && strategy == 2){
//                    ROS_ERROR("Fallback to euclidean distance.");
//                    release_mutex(&store_frontier_mutex, __FUNCTION__);
//                    return this->determine_goal_staying_alive_2(1, 1, available_distance, final_goal, count, robot_str_name, -1);
//                }
//                release_mutex(&store_frontier_mutex, __FUNCTION__);
//                ROS_ERROR("None of the %d checked frontiers is reachable! This shouldn't happen...", 8);
//                return false;
//            }

//            bool under_auction = false;
//            for(int k=0; !under_auction && k<frontiers_under_auction.size(); k++)
//                if(frontiers[i].id == frontiers_under_auction[k].id) {
//                    under_auction = true;
//                }
//            if(under_auction)
//                continue;

//            if (check_efficiency_of_goal(frontiers.at(i).x_coordinate, frontiers.at(i).y_coordinate) == true)
//            {
//                double distance;
//                double total_distance;
//                if(strategy == 1){
//                    // distance to next frontier
//                    double x1 = frontiers.at(i).x_coordinate - robotPose.getOrigin().getX();
//                    double y1 = frontiers.at(i).y_coordinate -  robotPose.getOrigin().getY();
//                    // distance from frontier to home base
//                    double x2 = robot_home_x - frontiers.at(i).x_coordinate;
//                    double y2 = robot_home_y -  frontiers.at(i).y_coordinate;
//                    total_distance = sqrt(x1 * x1 + y1 * y1) + sqrt(x2 * x2 + y2 * y2);
//                }
//                else if(strategy == 2){
//                    // distance to next frontier
//                    total_distance = trajectory_plan(frontiers.at(i).x_coordinate, frontiers.at(i).y_coordinate);
//                    if(total_distance < 0){
//                        ROS_ERROR("Failed to compute distance!");
//                        if(errors == 0)
//                            my_error_counter++;
//                        errors++;
//                        continue;
//                    }
//                    // distance from frontier to optimal ds
//                    distance = trajectory_plan(frontiers.at(i).x_coordinate, frontiers.at(i).y_coordinate, optimal_ds_x, optimal_ds_y);
//                    if(distance < 0){
//                        ROS_ERROR("Failed to compute distance!");
//                        if(errors == 0)
//                            my_error_counter++;
//                        errors++;
//                        continue;
//                    }
//                    total_distance += distance;

//                    if(total_distance < closest){
//                        closest = total_distance;
//                        dist_front = total_distance - distance;
//                        dist_home = distance;
//                    }

//                    // convert from cells to meters
//                    total_distance *= costmap_ros_->getCostmap()->getResolution();
//                }
//                else{
//                    ROS_ERROR("Wrong strategy, cannot compute distance to goal!");
//                    release_mutex(&store_frontier_mutex, __FUNCTION__);
//                    return false;
//                }

//                ROS_INFO("Distance to frontier and then home: %.2f",total_distance);
//                if(available_distance > total_distance)
//                {
//                    ROS_INFO("------------------------------------------------------------------");
//                    ROS_INFO("Determined frontier with ID: %d   at x: %.2f     y: %.2f   detected by Robot %d", frontiers.at(i).id, frontiers.at(i).x_coordinate, frontiers.at(i).y_coordinate, frontiers.at(i).detected_by_robot);

//                    final_goal->push_back(frontiers.at(i).x_coordinate);
//                    final_goal->push_back(frontiers.at(i).y_coordinate);
//                    final_goal->push_back(frontiers.at(i).detected_by_robot);
//                    final_goal->push_back(frontiers.at(i).id);
//                    
//                    robot_str_name->push_back(frontiers.at(i).detected_by_robot_str);
//                    
//                    my_selected_frontier = &frontiers.at(i);
//                    
//#ifndef QUICK_SELECTION
//            
//                    // robot position
//                    double robot_x = robotPose.getOrigin().getX();
//                    double robot_y = robotPose.getOrigin().getY();

//					// frontier position
//                    double frontier_x = frontiers.at(i).x_coordinate;
//                    double frontier_y = frontiers.at(i).y_coordinate;

//					// calculate d_g
//                    int d_g = trajectory_plan(frontier_x, frontier_y);

//                    // calculate d_gb
//                    int d_gb = trajectory_plan(frontier_x, frontier_y, robot_home_x, robot_home_y);

//                    // calculate d_gbe
//                    int d_gbe;
//                    if(my_energy_above_th)
//                    {
//                        d_gbe = -d_gb;
//                    }
//                    else
//                    {
//                        d_gbe = d_gb;
//                    }

//                    // calculate theta
//                    double theta_s = atan2(robot_last_y - robot_y, robot_last_x - robot_x);
//                    double theta_g = atan2(robot_y - frontier_y, robot_x - frontier_x);
//                    double theta = 1/M_PI * (M_PI - abs(abs(theta_s - theta_g) - M_PI));

//                    // calculate cost function
//                     my_bid = w1 * d_g + w2 * d_gb + w3 * d_gbe + w4 * theta;
//             
//#endif

//                    release_mutex(&store_frontier_mutex, __FUNCTION__);
//                    my_error_counter = 0;
//                    return true;
//                    
//                } else{
//                    ROS_ERROR("No frontier in energetic range (%.2f < %.2f + %.2f)", available_distance, dist_front * costmap_ros_->getCostmap()->getResolution(), dist_home * costmap_ros_->getCostmap()->getResolution());
//                    release_mutex(&store_frontier_mutex, __FUNCTION__);
//                    my_error_counter++;
//                    return false;
//                }
//            }
//        }
//    }

//    // look for a CLUSTER as goal
//    else if (mode == 2)
//    {
//        int cluster_vector_position = 0;

//        if(clusters.size() > 0)
//        {
//            for (int i = 0; i < clusters.size(); i++)
//            {
//                if(clusters.at(i).id == actual_cluster_id)
//                {
//                    if(clusters.at(i).cluster_element.size() > 0)
//                    {
//                        cluster_vector_position = i;
//                    }
//                    break;
//                }
//            }
//        }

//        ROS_INFO("Calculated vector position of cluster %d", actual_cluster_id);
//        /*
//         * Iterate over all clusters .... if cluster_vector_position is set
//         * also the clusters with a lower have to be checked if the frontier
//         * determination fails at clusters.at(cluster_vector_position). therefore
//         * a ring-buffer is operated to iterate also over lower clusters, since
//         * they might have changed.
//         */
//        int nothing_found_in_actual_cluster = 0;
//        int visited_clusters = 0;
//        for (int i = 0 + count; i < clusters.size(); i++)
//        {
//            i = i+ cluster_vector_position;
//            i = i % (clusters.size());
//            for (int j = 0; j < clusters.at(i).cluster_element.size(); j++)
//            {
//                if (check_efficiency_of_goal(clusters.at(i).cluster_element.at(j).x_coordinate, clusters.at(i).cluster_element.at(j).y_coordinate) == true)
//                {
//                    double distance;
//                    double total_distance;
//                    if(strategy == 1){
//                        // distance to cluster
//                        double x1 = clusters.at(i).cluster_element.at(j).x_coordinate - robotPose.getOrigin().getX();
//                        double y1 = clusters.at(i).cluster_element.at(j).y_coordinate - robotPose.getOrigin().getY();
//                        // distance from cluster to home base
//                        double x2 = robot_home_x - clusters.at(i).cluster_element.at(j).x_coordinate;
//                        double y2 = robot_home_y -  clusters.at(i).cluster_element.at(j).y_coordinate;
//                        total_distance = sqrt(x1 * x1 + y1 * y1) + sqrt(x2 * x2 + y2 * y2);
//                    }
//                    else if(strategy == 2){
//                        // distance to cluster
//                        total_distance = trajectory_plan(clusters.at(i).cluster_element.at(j).x_coordinate, clusters.at(i).cluster_element.at(j).y_coordinate);
//                        if(total_distance < 0){
//                            ROS_ERROR("Failed to compute distance!");
//                            errors++;
//                            continue;
//                        }
//                        // distance from cluster to home base
//                        distance = trajectory_plan(clusters.at(i).cluster_element.at(j).x_coordinate, clusters.at(i).cluster_element.at(j).y_coordinate, robot_home_x, robot_home_y);
//                        if(distance < 0){
//                            ROS_ERROR("Failed to compute distance!");
//                            errors++;
//                            continue;
//                        }
//                        total_distance += distance;
//                        // convert from cells to meters
//                        total_distance *= costmap_ros_->getCostmap()->getResolution();
//                    }
//                    else{
//                        ROS_ERROR("Wrong strategy, cannot compute distance to goal!");
//                        release_mutex(&store_frontier_mutex, __FUNCTION__);
//                        return false;
//                    }

//                    ROS_INFO("distance to cluster and then home: %f",total_distance);
//                    if(available_distance > total_distance)
//                    {
//                        ROS_INFO("------------------------------------------------------------------");
//                        ROS_INFO("Robot %d determined Goal: %d  at Clusters: %d",robot_name, (int)clusters.at(i).cluster_element.at(j).id, (int)clusters.at(i).id);

//                        final_goal->push_back(clusters.at(i).cluster_element.at(j).x_coordinate);
//                        final_goal->push_back(clusters.at(i).cluster_element.at(j).y_coordinate);
//                        final_goal->push_back(clusters.at(i).cluster_element.at(j).detected_by_robot);
//                        final_goal->push_back(clusters.at(i).cluster_element.at(j).id);

//                        // number of the cluster we operate in
//                        final_goal->push_back(clusters.at(i).id);
//                        robot_str_name->push_back(clusters.at(i).cluster_element.at(j).detected_by_robot_str);
//                        release_mutex(&store_frontier_mutex, __FUNCTION__);
//                        return true;
//                    }
//                }
//            }

//            nothing_found_in_actual_cluster ++;
//            visited_clusters ++;

//            if(nothing_found_in_actual_cluster == 1)
//            {
//                //start again at the beginning(closest cluster))
//                i=0;
//                cluster_vector_position = 0;
//            }

//            if(visited_clusters == clusters.size())
//            {
//                ROS_ERROR("No frontier in energetic range %.2f, going home for recharging", available_distance);
//                release_mutex(&store_frontier_mutex, __FUNCTION__);
//                return false;
//            }
//        }
//    }
//    
//    release_mutex(&store_frontier_mutex, __FUNCTION__); //probablt useless here...
//    
//}

/**
 * Check a frontier (or a cluster) if it is within reach of the robot considering its available energy
 * mode: 1=frontier, 2=cluster
 * strategy: 1=euclidean distance, 2=actual travel path
 */
//bool ExplorationPlanner::determine_goal_staying_alive(int mode, int strategy, double available_distance, std::vector<double> *final_goal, int count, std::vector<std::string> *robot_str_name, int actual_cluster_id)
//{
//    if (!costmap_ros_->getRobotPose(robotPose))
//    {
//            ROS_ERROR("Failed to get RobotPose");
//    }

//    // check if robot needs to go home right away
//    double dist_home;
//    double dist_front;
//    double closest = 9999;
//    if(strategy == 1){
//        double xh = robot_home_x - robotPose.getOrigin().getX();
//        double yh = robot_home_y - robotPose.getOrigin().getY();
//        dist_home = sqrt(xh * xh + yh * yh) * 1.2; // 1.2 is extra reserve, just in case
//    }
//    else if(strategy == 2){
//        //F
//        //dist_home = trajectory_plan(robot_home_x, robot_home_y) * costmap_ros_->getCostmap()->getResolution();
//        dist_home = trajectory_plan(robot_home_x, robot_home_y);
//    }
//    if(dist_home > 0 && dist_home >= available_distance)
//        return false;

//    // look for a FRONTIER as goal
//    int errors = 0;
//    if (mode == 1)
//    {
//        for (int i = 0 + count; i < frontiers.size(); i++)
//        {
//            //we only check the first 9 frontiers, because we only sorted the first 9 frontiers by efficiency.
//            if(i>8){
//                // if the distances could not be computed, try again using euclidean distances instead
//                if(errors == i && strategy == 2){
//                    ROS_ERROR("Fallback to euclidean distance.");
//                    return this->determine_goal_staying_alive(1, 1, available_distance, final_goal, count, robot_str_name, -1);
//                }
//                return false;
//            }

//            if (check_efficiency_of_goal(frontiers.at(i).x_coordinate, frontiers.at(i).y_coordinate) == true)
//            {
//                double distance;
//                double total_distance;
//                if(strategy == 1){
//                    // distance to next frontier
//                    double x1 = frontiers.at(i).x_coordinate - robotPose.getOrigin().getX();
//                    double y1 = frontiers.at(i).y_coordinate -  robotPose.getOrigin().getY();
//                    // distance from frontier to home base
//                    double x2 = robot_home_x - frontiers.at(i).x_coordinate;
//                    double y2 = robot_home_y -  frontiers.at(i).y_coordinate;
//                    total_distance = sqrt(x1 * x1 + y1 * y1) + sqrt(x2 * x2 + y2 * y2);
//                }
//                else if(strategy == 2){
//                    // distance to next frontier
//                    total_distance = trajectory_plan(frontiers.at(i).x_coordinate, frontiers.at(i).y_coordinate);
//                    if(total_distance < 0){
//                        ROS_ERROR("Failed to compute distance!");
//                        errors++;
//                        continue;
//                    }
//                    // distance from frontier to home base
//                    distance = trajectory_plan(frontiers.at(i).x_coordinate, frontiers.at(i).y_coordinate, robot_home_x, robot_home_y);
//                    if(distance < 0){
//                        ROS_ERROR("Failed to compute distance!");
//                        errors++;
//                        continue;
//                    }
//                    total_distance += distance;

//                    if(total_distance < closest){
//                        closest = total_distance;
//                        dist_front = total_distance - distance;
//                        dist_home = distance;
//                    }

//                    // convert from cells to meters
//                    total_distance *= costmap_ros_->getCostmap()->getResolution();
//                }
//                else{
//                    ROS_ERROR("Wrong strategy, cannot compute distance to goal!");
//                    return false;
//                }

//                ROS_INFO("Distance to frontier and then home: %.2f",total_distance);
//                if(available_distance > total_distance)
//                {
//                    ROS_INFO("------------------------------------------------------------------");
//                    ROS_INFO("Determined frontier with ID: %d   at x: %.2f     y: %.2f   detected by Robot %d", frontiers.at(i).id, frontiers.at(i).x_coordinate, frontiers.at(i).y_coordinate, frontiers.at(i).detected_by_robot);

//                    final_goal->push_back(frontiers.at(i).x_coordinate);
//                    final_goal->push_back(frontiers.at(i).y_coordinate);
//                    final_goal->push_back(frontiers.at(i).detected_by_robot);
//                    final_goal->push_back(frontiers.at(i).id);

//                    robot_str_name->push_back(frontiers.at(i).detected_by_robot_str);
//                    return true;
//                }else{
//                    ROS_INFO("No frontier in energetic range (%.2f < %.2f + %.2f)", available_distance, dist_front * costmap_ros_->getCostmap()->getResolution(), dist_home * costmap_ros_->getCostmap()->getResolution());
//                    return false;
//                }
//            }
//        }
//    }

//    // look for a CLUSTER as goal
//    else if (mode == 2)
//    {
//        int cluster_vector_position = 0;

//        if(clusters.size() > 0)
//        {
//            for (int i = 0; i < clusters.size(); i++)
//            {
//                if(clusters.at(i).id == actual_cluster_id)
//                {
//                    if(clusters.at(i).cluster_element.size() > 0)
//                    {
//                        cluster_vector_position = i;
//                    }
//                    break;
//                }
//            }
//        }

//        ROS_INFO("Calculated vector position of cluster %d", actual_cluster_id);
//        /*
//         * Iterate over all clusters .... if cluster_vector_position is set
//         * also the clusters with a lower have to be checked if the frontier
//         * determination fails at clusters.at(cluster_vector_position). therefore
//         * a ring-buffer is operated to iterate also over lower clusters, since
//         * they might have changed.
//         */
//        int nothing_found_in_actual_cluster = 0;
//        int visited_clusters = 0;
//        for (int i = 0 + count; i < clusters.size(); i++)
//        {
//            i = i+ cluster_vector_position;
//            i = i % (clusters.size());
//            for (int j = 0; j < clusters.at(i).cluster_element.size(); j++)
//            {
//                if (check_efficiency_of_goal(clusters.at(i).cluster_element.at(j).x_coordinate, clusters.at(i).cluster_element.at(j).y_coordinate) == true)
//                {
//                    double distance;
//                    double total_distance;
//                    if(strategy == 1){
//                        // distance to cluster
//                        double x1 = clusters.at(i).cluster_element.at(j).x_coordinate - robotPose.getOrigin().getX();
//                        double y1 = clusters.at(i).cluster_element.at(j).y_coordinate - robotPose.getOrigin().getY();
//                        // distance from cluster to home base
//                        double x2 = robot_home_x - clusters.at(i).cluster_element.at(j).x_coordinate;
//                        double y2 = robot_home_y -  clusters.at(i).cluster_element.at(j).y_coordinate;
//                        total_distance = sqrt(x1 * x1 + y1 * y1) + sqrt(x2 * x2 + y2 * y2);
//                    }
//                    else if(strategy == 2){
//                        // distance to cluster
//                        total_distance = trajectory_plan(clusters.at(i).cluster_element.at(j).x_coordinate, clusters.at(i).cluster_element.at(j).y_coordinate);
//                        if(total_distance < 0){
//                            ROS_ERROR("Failed to compute distance!");
//                            errors++;
//                            continue;
//                        }
//                        // distance from cluster to home base
//                        distance = trajectory_plan(clusters.at(i).cluster_element.at(j).x_coordinate, clusters.at(i).cluster_element.at(j).y_coordinate, robot_home_x, robot_home_y);
//                        if(distance < 0){
//                            ROS_ERROR("Failed to compute distance!");
//                            errors++;
//                            continue;
//                        }
//                        total_distance += distance;
//                        // convert from cells to meters
//                        total_distance *= costmap_ros_->getCostmap()->getResolution();
//                    }
//                    else{
//                        ROS_ERROR("Wrong strategy, cannot compute distance to goal!");
//                        return false;
//                    }

//                    ROS_INFO("distance to cluster and then home: %f",total_distance);
//                    if(available_distance > total_distance)
//                    {
//                        ROS_INFO("------------------------------------------------------------------");
//                        ROS_INFO("Robot %d determined Goal: %d  at Clusters: %d",robot_name, (int)clusters.at(i).cluster_element.at(j).id, (int)clusters.at(i).id);

//                        final_goal->push_back(clusters.at(i).cluster_element.at(j).x_coordinate);
//                        final_goal->push_back(clusters.at(i).cluster_element.at(j).y_coordinate);
//                        final_goal->push_back(clusters.at(i).cluster_element.at(j).detected_by_robot);
//                        final_goal->push_back(clusters.at(i).cluster_element.at(j).id);

//                        // number of the cluster we operate in
//                        final_goal->push_back(clusters.at(i).id);
//                        robot_str_name->push_back(clusters.at(i).cluster_element.at(j).detected_by_robot_str);
//                        return true;
//                    }
//                }
//            }

//            nothing_found_in_actual_cluster ++;
//            visited_clusters ++;

//            if(nothing_found_in_actual_cluster == 1)
//            {
//                //start again at the beginning(closest cluster))
//                i=0;
//                cluster_vector_position = 0;
//            }

//            if(visited_clusters == clusters.size())
//            {
//                ROS_ERROR("No frontier in energetic range %.2f, going home for recharging", available_distance);
//                return false;
//            }
//        }
//    }
//}

//bool ExplorationPlanner::my_determine_goal_staying_alive(int mode, int strategy, double available_distance, std::vector<double> *final_goal, int count, std::vector<std::string> *robot_str_name, int actual_cluster_id, bool energy_above_th, int w1, int w2, int w3, int w4)
//{
//    
//    ros::Time start_time;
//    selection_time = 0;
//    number_of_frontiers = 0;
//    frontier_selected = false;
//    start_time = ros::Time::now();
//    winner_of_auction = true;
//    this->available_distance = available_distance;

//    if (frontiers.size() <= 0 && clusters.size() <= 0)
//    {
//        ROS_ERROR("No frontier/cluster available");
//    } 
//    
//    sorted_frontiers.clear();
//    
//    //TODO move to a separate function that is called by explorer, since in case of error (when my_... is recalled by itself), this code otherwise is re-executed every time...
//    ROS_DEBUG("frontiers size: %u", frontiers.size());
//    if(APPROACH == 0)
//        sorted_frontiers = frontiers;
//    else if(APPROACH == 1)
//        sorted_frontiers = frontiers;
//    else if(APPROACH == 2)
//        my_sort_cost_2(energy_above_th, w1, w2, w3, w4);
//    else if(APPROACH == 3)
//        my_sort_cost_3(energy_above_th, w1, w2, w3, w4);
//    else if(APPROACH == 4)
//        my_select_4(available_distance, energy_above_th, w1, w2, w3, w4, final_goal, robot_str_name);
//    else {
//        ROS_ERROR("Invalid approach!");
//        sorted_frontiers = frontiers;
//    }
//    
//    selection_time = (ros::Time::now() - start_time).toSec();
//    number_of_frontiers = frontiers.size();

//    acquire_mutex(&store_frontier_mutex, __FUNCTION__);
//    if(frontier_selected)
//        for(int i=0; i < sorted_frontiers.size(); i++)
//        {

//            //start auction
//            my_selected_frontier = &sorted_frontiers.at(i);
//            my_bid = sorted_frontiers.at(i).cost;
//            ROS_INFO("start frontier negotiation!");
//            my_negotiate();
//     
//            for(int i = 0; i < auction_timeout/0.1; i++) {
//                ros::Duration(0.1).sleep();
//                ros::spinOnce();
//            }
//            
//            if(!winner_of_auction) {
//                ROS_INFO("frontier under auction: skip");
//                continue;
//            }

//            ROS_INFO("frontier selected");
//            
//            final_goal->push_back(my_selected_frontier->x_coordinate);
//            final_goal->push_back(my_selected_frontier->y_coordinate);
//            ROS_INFO("selected goal: %.2f, %.2f", my_selected_frontier->x_coordinate, my_selected_frontier->y_coordinate);
//            final_goal->push_back(my_selected_frontier->detected_by_robot);
//            final_goal->push_back(my_selected_frontier->id);
//            
//            robot_str_name->push_back(robot_name_str); 
//            
//            frontiers_under_auction.clear();
//            my_error_counter = 0;
//            errors = 0;
//            
//            adhoc_communication::ExpFrontier negotiation_list;
//            adhoc_communication::ExpFrontierElement negotiation_element;
//            negotiation_element.detected_by_robot = robot_name;
//            negotiation_element.x_coordinate = my_selected_frontier->x_coordinate;
//            negotiation_element.y_coordinate = my_selected_frontier->y_coordinate;
//            negotiation_element.id = my_selected_frontier->id;
//                
//            negotiation_list.frontier_element.push_back(negotiation_element);

//            my_sendToMulticast("mc_", negotiation_list, "send_next_robot_goal");
//            
//            release_mutex(&store_frontier_mutex, __FUNCTION__);
//            return true;
//        }

//    frontier_selected = false;
//    release_mutex(&store_frontier_mutex, __FUNCTION__);
//    return false;

//}

//void ExplorationPlanner::robot_amcl_position_callback(geometry_msgs::PoseWithCovarianceStamped

/*
bool ExplorationPlanner::get_robot_position(double *x, double *y) { //F WRONG!!!!!!!!!
    
    if(costmap_ros_ == NULL) {
        ROS_ERROR("NULL!!!");
        return false;   
    }
    tf::Stamped<tf::Pose> robot_pose;
    if(costmap_global_ros_->getRobotPose(robot_pose)) {
        *x = robot_pose.getOrigin().getX() * costmap_ros_->getCostmap()->getResolution();
        *y = robot_pose.getOrigin().getY() * costmap_ros_->getCostmap()->getResolution();
        return true;
    }
    
    return false;
}
*/


bool ExplorationPlanner::auctioning(std::vector<double> *final_goal, std::vector<int> *clusters_available_in_pool, std::vector<std::string> *robot_str_name)
{

    ROS_INFO("Start Auctioning");

   /*
    * Check if Auction is running and wait until it is finished
    */
    int timer_count = 0;
    number_of_auction_bids_received = 0;
    std::string robo_name;

    float r = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
    float wait_if_auction_runs = r * 2;
//    wait_if_auction_runs = wait_if_auction_runs - robot_name; // FIXME
    if(wait_if_auction_runs < 0)
        wait_if_auction_runs = wait_if_auction_runs * (-1);

    ROS_INFO("Waiting %f second if a auction is running",wait_if_auction_runs);
    ros::Duration(wait_if_auction_runs).sleep();

    while(auction_running)
    {
        ROS_INFO("Waiting %f second because an auction is currently running",wait_if_auction_runs);
        ros::Duration(wait_if_auction_runs).sleep();
        float wait_if_auction_runs = r * 2;
        wait_if_auction_runs = wait_if_auction_runs - robot_name;
        if(wait_if_auction_runs < 0)
            wait_if_auction_runs = wait_if_auction_runs * (-1);

    }

    //EDIT Peter : Removed this wait part to speed up the process!

    /*
    if(robot_name != 0)
        auction_finished = false;
    //why here sleep so long ?
    while(auction_finished == false && timer_count <= 20)
    {
        timer_count++;
        ros::Duration(1).sleep();
    }*/

    /*
     * If no auction is running ... clear the auction vector and
     * start a new auction.
     */
    //how do i know that no auction is running ??
    //EDIT Peter: New bool to check if auction is running!

    if(auction_running)
    {
        ROS_INFO("Auction is running, leaving method");
        return false;
    }
    else
    {
        ROS_INFO("No auction is running, clearing");
        auction.clear();
    }


//    adhoc_communication::AuctionStatus auction_status;
    adhoc_communication::ExpAuction reqest_clusters, auction_msg;
//    auction_status.start_auction = true;
//    auction_status.auction_finished = false;

    auction_msg.start_auction = true;
    auction_msg.auction_finished = false;
    auction_msg.auction_status_message = true;

    if(robot_prefix_empty_param == false)
    {
    std::stringstream ss;
    ss << robot_name;
    std::string prefix = "";
    robo_name = prefix.append(ss.str());

    auction_msg.robot_name = robo_name;
    }else
    {
        auction_msg.robot_name = robot_str;
        robo_name = robot_str;
    }
    /*
     * visualize all cluster elements
     */

//    for(int i = 0; i < clusters.size(); i++)
//    {
//        ROS_INFO("---- Cluster %d ----", clusters.at(i).id);
//        std::string requested_clusters;
//        for(int j = 0; j < clusters.at(i).cluster_element.size(); j++)
//        {
//            requested_clusters.append(NumberToString((int)clusters.at(i).cluster_element.at(j).id));
//            requested_clusters.append(", ");
//        }
//        ROS_INFO("Cluster ids: %s", requested_clusters.c_str());
//    }



    for(int i = 0; i < clusters.size(); i++)
    {
        adhoc_communication::ExpCluster cluster_request;


        for(int j = 0; j < clusters.at(i).cluster_element.size(); j++)
        {
            adhoc_communication::ExpClusterElement cluster_request_element;
            if(robot_prefix_empty_param == true)
            {
                cluster_request_element.id = clusters.at(i).cluster_element.at(j).id;
                cluster_request_element.detected_by_robot_str = clusters.at(i).cluster_element.at(j).detected_by_robot_str;
            }else
            {
                cluster_request_element.id = clusters.at(i).cluster_element.at(j).id;
            }
            cluster_request.ids_contained.push_back(cluster_request_element);
        }
//        auction_status.requested_clusters.push_back(cluster_request);
        auction_msg.requested_clusters.push_back(cluster_request);
    }

    if(robot_prefix_empty_param == true)
    {
        auction_msg.auction_id = auction_id_number;
    }else
    {
        auction_msg.auction_id = 10000*robot_name + auction_id_number;
    }

    /*
     * Following code is to visualize the message being send to the other robots
     */
//    for(int i = 0; i< auction_msg.requested_clusters.size(); i++)
//    {
//        adhoc_communication::Cluster cluster_req;
//        cluster_req = auction_msg.requested_clusters.at(i);
//        ROS_INFO("----------------------- REQUEST %d ---------------------------", i);
//        std::string requested_clusters;
//        for(int j = 0; j < cluster_req.ids_contained.size(); j++)
//        {
//            requested_clusters.append(NumberToString((int)cluster_req.ids_contained.at(j)));
//            requested_clusters.append(", ");
//        }
//        ROS_INFO("Auction with auction number: %d", auction_msg.auction_id);
//        ROS_INFO("Requested ids: %s", requested_clusters.c_str());
//    }


    std::string numbers_of_operating_robots;

    ROS_INFO("Robot %d starting an auction", robot_name);

//    if(first_run == true)
//    {
//        pub_auctioning_first.publish(auction_msg);
//    }else
//    {
        sendToMulticastAuction("mc_", auction_msg, "auction");
//    }



    /*
     * Wait for results of others
     */


       //EDIT Peter: I do not know how many answers i get
        /*
    nh.param("/robots_in_simulation",number_of_robots, 1);
    number_of_robots = 2; // To test with two robots if parameter does not work
    ROS_INFO("++++++++++ number of robots: %d ++++++++++", number_of_robots);
    */

//    ros::NodeHandle nh_robots;
//    nh_robots.param<std::string>("/robots_in_simulation",numbers_of_operating_robots, "1");
//    number_of_robots = atoi(numbers_of_operating_robots.c_str());
//    ROS_INFO("++++++++++ number of robots: %d ++++++++++", number_of_robots);

    /*
     * Number of responding robots is one less since the robot itself does not
     * respond to its own request
     */
    //ROS_INFO("Waiting for results from %d robots", number_of_robots);
    //wait 4 seconds to receive bids
        ros::Duration(3).sleep();
        /*
    while(timer_count < 50 && number_of_auction_bids_received < number_of_robots -1)
    {
        ROS_INFO("Waiting for results from %d robots", number_of_robots);
        timer_count++;
        ros::Duration(0.2).sleep();
    }*/
    ROS_INFO("number of auction bids received: %d", number_of_auction_bids_received);


    if(number_of_auction_bids_received > 0)
        number_of_completed_auctions++;
    else
        number_of_uncompleted_auctions++;
    /*if(number_of_auction_bids_received < number_of_robots-1)
    {
        ROS_ERROR("Wait for other robots timer exceeded");
        number_of_uncompleted_auctions++;
    }else if(number_of_auction_bids_received >= number_of_robots - 1)
    {
        number_of_completed_auctions++;
    }*/


    /*
     * Select the best cluster for yourself
     */
    ROS_INFO("Selecting the most attractive cluster");
    bool cluster_selected_flag = selectClusterBasedOnAuction(final_goal, clusters_available_in_pool, robot_str_name);

    /*
     * Stop the auction
     */
    ROS_INFO("Stop the auction");
//    auction_status.start_auction = false;
//    auction_status.auction_finished = true;

    auction_msg.start_auction = false;
    auction_msg.auction_finished = true;
    auction_msg.auction_status_message = true;
    auction_msg.robot_name = robo_name;
    auction_msg.requested_clusters.clear();

    if(cluster_selected_flag == true)// && final_goal->size() >= 4)
    {
        /*
         * Tell the others which cluster was selected
         */
        std::vector<transform_point_t> occupied_ids;
        clusterIdToElementIds(final_goal->at(4), &occupied_ids);
        for(int i = 0; i < occupied_ids.size(); i++)
        {
//            auction_status.occupied_ids.push_back(occupied_ids.at(i));
            adhoc_communication::ExpAuctionElement auction_element;
            auction_element.id = occupied_ids.at(i).id;
            auction_element.detected_by_robot_str = occupied_ids.at(i).robot_str;
            auction_msg.occupied_ids.push_back(auction_element);
        }
    }
//    else
//    {
//        ROS_ERROR("No Goal Selected from selectClusterBasedOnAuction");
//        cluster_selected_flag = false;
//    }

//    if(first_run == true)
//    {
//    pub_auctioning_status.publish(auction_status);
//        pub_auctioning_first.publish(auction_msg);
//    }else
//    {
        sendToMulticastAuction("mc_", auction_msg, "auction");
//    }

    first_run = false;
    auction_id_number++;

    ROS_INFO("return %d", cluster_selected_flag);
    return (cluster_selected_flag);
}

void ExplorationPlanner::clearVisitedAndSeenFrontiersFromClusters()
{
    ROS_INFO("Clear VisitedAndSeenFrontiers from Cluster");
    std::vector<int> goals_to_clear;



    for(unsigned int i = 0; i < clusters.size(); i++)
    {
        for(unsigned int j = 0; j < clusters.at(i).cluster_element.size(); j++)
        {
            /* Now iterate over all frontiers and check if cluster elements are
             * still available in the frontier vector
             */
            bool cluster_still_valid = false;
            for(unsigned int m = 0; m < frontiers.size(); m++)
            {
                if(clusters.at(i).cluster_element.at(j).id == frontiers.at(m).id)
                {
                    /*Cluster is still valid, do not clear it*/
                    cluster_still_valid = true;
                    break;
                }
            }
            if(cluster_still_valid == false)
            {
                clusters.at(i).cluster_element.erase(clusters.at(i).cluster_element.begin() +j);
                if(j > 0)
                   j--;
            }
        }
    }

    for(unsigned int i = 0; i < clusters.size(); i++)
    {
        if(clusters.at(i).cluster_element.size() <= 0)
        {
            clusters.erase(clusters.begin() + i);
        }
    }


//    if(visited_frontiers.size() > 1)
//    {
//        for (int i = 1; i < visited_frontiers.size(); i++)
//        {
//            bool found_flag = false;
//            for (int j = 0; j < clusters.size(); j++)
//            {
//                for(int n = 0; n < clusters.at(j).cluster_element.size(); n++)
//                {
//                    double diff_x = visited_frontiers.at(i).x_coordinate - clusters.at(j).cluster_element.at(n).x_coordinate;
//                    double diff_y = visited_frontiers.at(i).y_coordinate - clusters.at(j).cluster_element.at(n).y_coordinate;
//
//                    if (fabs(diff_x) <= MAX_GOAL_RANGE && fabs(diff_y) <= MAX_GOAL_RANGE)
//                    {
//                        ROS_DEBUG("Erasing visited frontier %d from cluster", clusters.at(j).cluster_element.at(n).id);
//                        clusters.at(j).cluster_element.erase(clusters.at(j).cluster_element.begin()+n);
//                        if(n > 0)
//                        {
//                            n --;
//                        }
//                        found_flag = true;
//    //                    break;
//                    }
//                }
//    //            if(found_flag == true)
//    //            {
//    //                break;
//    //            }
//            }
//        }
//    }

//    if(seen_frontier_list.size() > 1)
//    {
//        for (int i = 1; i < seen_frontier_list.size(); i++)
//        {
//            bool found_flag = false;
//            for (int j = 0; j < clusters.size(); j++)
//            {
//                for(int n = 0; n < clusters.at(j).cluster_element.size(); n++)
//                {
//                    double diff_x = seen_frontier_list.at(i).x_coordinate - clusters.at(j).cluster_element.at(n).x_coordinate;
//                    double diff_y = seen_frontier_list.at(i).y_coordinate - clusters.at(j).cluster_element.at(n).y_coordinate;
//
//                    if (fabs(diff_x) <= MAX_GOAL_RANGE*2 && fabs(diff_y) <= MAX_GOAL_RANGE*2)
//                    {
//                        ROS_DEBUG("Erasing seen frontier %d from cluster", clusters.at(j).cluster_element.at(n).id);
//                        clusters.at(j).cluster_element.erase(clusters.at(j).cluster_element.begin()+n);
//                        if(n > 0)
//                        {
//                            n --;
//                        }
//                        found_flag = true;
//    //                    break;
//                    }
//                }
//    //            if(found_flag == true)
//    //            {
//    //                break;
//    //            }
//            }
//        }
//    }
    ROS_INFO("Done");
}

/**
 * Check if the next goal is efficient enough to steer the robot to it.
 * If it is a goal, which has previously be seen, it is not required
 * to visit this goal again.
 * make sure that there are no failures in calculation! Therefore
 * this plausibility check is done. Only values of less then 50m
 * are valid. (Calculation of coordinates in the costmap often
 * produce very big values which are miss-interpretations)
 */
bool ExplorationPlanner::check_efficiency_of_goal(double x, double y)
{
    double diff_home_x = visited_frontiers.at(0).x_coordinate - x;
    double diff_home_y = visited_frontiers.at(0).y_coordinate - y;

    if (fabs(diff_home_x) <= MAX_DISTANCE && fabs(diff_home_y) <= MAX_DISTANCE)
    {
        for (unsigned int i = 1; i < visited_frontiers.size(); i++)
        {
            /*
             * Calculate the distance between all previously seen goals and the new
             * found frontier!!
             */
            double diff_x = visited_frontiers.at(i).x_coordinate - x;
            double diff_y = visited_frontiers.at(i).y_coordinate - y;

            if (fabs(diff_x) <= MAX_GOAL_RANGE && fabs(diff_y) <= MAX_GOAL_RANGE) {
                ROS_DEBUG("x: %f  y: %f too close to visited at x: %f   y: %f   diff_x: %f   diff_y: %f", x, y, visited_frontiers.at(i).x_coordinate, visited_frontiers.at(i).y_coordinate, diff_x, diff_y);
                return false;
            }
        }
        for (unsigned int i = 0; i < unreachable_frontiers.size(); i++)
        {
            /*
             * Calculate the distance between all previously seen goals and the new
             * found frontier!!
             */
            double diff_x = unreachable_frontiers.at(i).x_coordinate - x;
            double diff_y = unreachable_frontiers.at(i).y_coordinate - y;

            if (fabs(diff_x) <= MAX_GOAL_RANGE && fabs(diff_y) <= MAX_GOAL_RANGE) {
                ROS_DEBUG("x: %f  y: %f too close to unreachable at x: %f   y: %f   diff_x: %f   diff_y: %f", x, y, unreachable_frontiers.at(i).x_coordinate, unreachable_frontiers.at(i).y_coordinate, diff_x, diff_y);
                return false;
            }
        }
        return true;
    }
    else
    {
        ROS_WARN("OUT OF HOME RANGE");
        return false;
    }
}


int ExplorationPlanner::checkClustersID(adhoc_communication::ExpCluster cluster_to_check)
{
//    ROS_INFO("Check for cluster id");
      std::vector<compare_pair_t> compare_clusters;

//        ROS_INFO("------------------- Checking ----------------------");
        std::string requested_clusters;

//        for(int j = 0; j < cluster_to_check.ids_contained.size(); j++)
//        {
//            requested_clusters.append(NumberToString((int)cluster_to_check.ids_contained.at(j)));
//            requested_clusters.append(", ");
//        }
//        ROS_INFO("%s", requested_clusters.c_str());




    for(unsigned int j = 0; j < clusters.size(); j++)
    {
        double same_id_found = 0;
//        ROS_INFO("--------------------------------------------------------------");
        for(unsigned int n = 0; n < clusters.at(j).cluster_element.size(); n++)
        {
//            for(int m= 0; m < clusters.at(j).cluster_element.size(); m++)
//            {
//                ROS_INFO("Ids in cluster to check with: %d",clusters.at(j).cluster_element.at(m).id);
//            }
//            ROS_INFO("Ids in cluster to check with: %d",clusters.at(j).cluster_element.at(n).id);
            for(unsigned int i = 0; i < cluster_to_check.ids_contained.size(); i++)
            {
                if(robot_prefix_empty_param == true)
                {
//                    ROS_INFO("Cluster id: %d  robot: %s     msg id: %d  robot: %s",clusters.at(j).cluster_element.at(n).id, clusters.at(j).cluster_element.at(n).detected_by_robot_str, cluster_to_check.ids_contained.at(i).id, cluster_to_check.ids_contained.at(i).detected_by_robot_str);
                    if(clusters.at(j).cluster_element.at(n).id == cluster_to_check.ids_contained.at(i).id && clusters.at(j).cluster_element.at(n).detected_by_robot_str.compare(cluster_to_check.ids_contained.at(i).detected_by_robot_str) == 0)
                    {
                        same_id_found++;
                    }
                }else
                {
                    if(clusters.at(j).cluster_element.at(n).id == cluster_to_check.ids_contained.at(i).id)
                    {
                        same_id_found++;
                    }
                }
            }
        }
        if(same_id_found > clusters.at(j).cluster_element.size() * 0.4)
        {
//            ROS_INFO("CLUSTER ID FOUND: %d", clusters.at(j).id);
            return (clusters.at(j).id);
        }
        else
        {
//            ROS_ERROR("NO MATCHING CLUSTER FOUND!!!");
        }
//        ROS_INFO("---------------------------------------------------");

//        ROS_INFO("j: %d   cluster_id: %d", j, clusters.at(j).id);
//        compare_pair_t compare_pair;
//        compare_pair.identical_ids = same_id_found;
//        compare_pair.cluster_id = clusters.at(j).id;
//        compare_clusters.push_back(compare_pair);
//        ROS_INFO("Same IDs found: %f   elements*0.7: %f", same_id_found,clusters.at(j).cluster_element.size() * 0.7);


//            if(same_id_found > clusters.at(j).cluster_element.size() * 0.8)
//            {
//                bool previously_detected = false;
//                for(int m = 0; m < id_previously_detected.size(); m++)
//                {
//                    if(clusters.at(j).id == id_previously_detected.at(m))
//                    {
//                        previously_detected = true;
//                        break;
//                    }
//                }
//                if(previously_detected == false)
//                {
//                    id_previously_detected.push_back(clusters.at(j).id);
//                    return (clusters.at(j).id);
//                }
//            }
    }

    /*
     * Sort compare_pairs and select the one with the highest similarity
     */
//    if(compare_clusters.size() > 0)
//    {
//
//        std::sort(compare_clusters.begin(), compare_clusters.end(), sortCompareElements);
////        for(int i = 0; i < compare_clusters.size(); i++)
////        {
////            ROS_INFO("identical: %d    cluster id: %d", compare_clusters.at(i).identical_ids, compare_clusters.at(i).cluster_id);
////        }
////        ROS_INFO("Selected identical: %d    cluster id: %d", compare_clusters.front().identical_ids, compare_clusters.front().cluster_id);
//        return(compare_clusters.front().cluster_id);
//    }else
//    {
//        ROS_INFO("Compare Cluster empty");
//        return (-1);
//    }
    return (-1);
}

void ExplorationPlanner::auctionCallback(const adhoc_communication::ExpAuction::ConstPtr& msg)
{
    auction_running = true;
    //ROS_ERROR("CALLING AUCTION CALLBACK!!!!!!!!!!!!");
    int robots_int_name = -1;

    bool same_robot = false;
    if(robot_prefix_empty_param == true)
    {
        if(robot_str.compare(msg.get()->robot_name.c_str()) == 0)
        {
            same_robot = true;
        }
    }else
    {
        robots_int_name = atoi(msg.get()->robot_name.c_str());
        ROS_INFO("Robot name: %d      int_name: %d",robot_name, robots_int_name);
        if(robots_int_name == robot_name)
        {
            same_robot = true;
        }
    }

    if(same_robot == false)
    {
        /*
        * Cluster all available frontiers to be able to compare clusters
        * with other robots
        */

        /*
         * TODO
         * Check if following code is necessary or if simple navigation needs to
         * periodically update the frontiers in the frontier thread.
         */
        transformToOwnCoordinates_frontiers();
        transformToOwnCoordinates_visited_frontiers();

        clearVisitedFrontiers();
        clearUnreachableFrontiers();
        clearSeenFrontiers(costmap_global_ros_);

        clearVisitedAndSeenFrontiersFromClusters();
        clusterFrontiers();


//        visualize_Cluster_Cells();

        if(msg.get()->auction_status_message == true)
        {
            ROS_INFO("Calling Auction Status");

            /*
             * Visualize requested ids
             */

            for(unsigned int i = 0; i < msg.get()->requested_clusters.size(); i++)
            {
                adhoc_communication::ExpCluster cluster_req;
                cluster_req = msg.get()->requested_clusters.at(i);
                std::string requested_clusters;
                for(unsigned int j = 0; j < cluster_req.ids_contained.size(); j++)
                {
                    if(j >= 6)
                    {
                        break;
                    }
                    requested_clusters.append(NumberToString((int)cluster_req.ids_contained.at(j).id));
                    requested_clusters.append(", ");
                }
                ROS_INFO("Requested ids: %s from robot: %s", requested_clusters.c_str(), msg.get()->robot_name.c_str());
            }



            auction_start = msg.get()->start_auction;
            auction_finished = msg.get()->auction_finished;
            adhoc_communication::ExpCluster occupied_ids;
            std::vector<requested_cluster_t> requested_cluster_ids;


            /*
             * Grep all occupied ids and try to convert them into clusters in own
             * coordinate system. This ensures that all robots in the system know
             * which clusters had been occupied by others and do not select them
             * twice.
             */
            if(msg.get()->occupied_ids.size() > 0 || msg.get()->requested_clusters.size() > 0)
            {
                for(int i = 0; i < msg.get()->occupied_ids.size(); i++)
                {
                    adhoc_communication::ExpClusterElement cluster_element;

                    cluster_element.id = msg.get()->occupied_ids.at(i).id;
                    cluster_element.detected_by_robot_str = msg.get()->occupied_ids.at(i).detected_by_robot_str;

                    occupied_ids.ids_contained.push_back(cluster_element);
                }
                int occupied_cluster_id = checkClustersID(occupied_ids);
                ROS_INFO("Check occupied cluster to be the same. %d", occupied_cluster_id);

                if(occupied_cluster_id >=0)
                {
                    ROS_INFO("Adding occupied cluster: %d", occupied_cluster_id);
                    already_used_ids.push_back(occupied_cluster_id);
                }else
                {
                    /* Store undetected Clusters in a struct for later processing */
                    ROS_INFO("Adding occupied cluster as unrecognized!");
                    unrecognized_occupied_clusters.push_back(occupied_ids);
                }
                /*
                 * Now read from unrecognized cluster vector and try to convert
                 * these ids to a valid cluster to know which had already been
                 * occupied.
                 */
                for(unsigned int i = 0; i < unrecognized_occupied_clusters.size(); i++)
                {
                    int unrecognized_occupied_cluster_id = checkClustersID(unrecognized_occupied_clusters.at(i));
                    if(unrecognized_occupied_cluster_id >=0)
                    {
                        ROS_INFO("Unrecognized cluster: %d converted", unrecognized_occupied_cluster_id);
                        already_used_ids.push_back(unrecognized_occupied_cluster_id);
                        unrecognized_occupied_clusters.erase(unrecognized_occupied_clusters.begin() + i);

                        if(i > 0)
                            i--;
                    }
                }


                /*
                 * Grep the requested clusters to know which ones to answer to.
                 */
                for(unsigned int i = 0; i < msg.get()->requested_clusters.size(); i++)
                {
                    adhoc_communication::ExpCluster requested_cluster;
                    requested_cluster = msg.get()->requested_clusters.at(i);

                    int check_cluster_id = checkClustersID(requested_cluster);
                    if(check_cluster_id >= 0)
                    {
                        requested_cluster_t new_cluster_request;
                        for(unsigned int j = 0; j < requested_cluster.ids_contained.size(); j++)
                        {
                            transform_point_t cluster_element_point;
                            if(robot_prefix_empty_param == true)
                            {
                                cluster_element_point.id = requested_cluster.ids_contained.at(j).id;
                                cluster_element_point.robot_str = requested_cluster.ids_contained.at(j).detected_by_robot_str;
                            }else
                            {
                                cluster_element_point.id = requested_cluster.ids_contained.at(j).id;
                            }

                            new_cluster_request.requested_ids.push_back(cluster_element_point);
                        }
                        new_cluster_request.own_cluster_id = check_cluster_id;

                        requested_cluster_ids.push_back(new_cluster_request);
                    }else
                    {
                        ROS_WARN("No Matching Cluster Detected");
                    }
                }
            }


            if(auction_start == true)
            {
                start_thr_auction = true;
                thr_auction_status = boost::thread(&ExplorationPlanner::respondToAuction, this, requested_cluster_ids, msg.get()->auction_id);
            }

        }else if(msg.get()->auction_status_message == false)
        {
            bool continue_auction = false;
            if(robot_prefix_empty_param == true)
            {
                if(msg.get()->auction_id == auction_id_number)
                {
                    continue_auction = true;
                }
            }else
            {
                if(msg.get()->auction_id == 10000*robot_name + auction_id_number)
                {
                    continue_auction = true;
                }
            }


            if(continue_auction == true)
            {
    //            ROS_INFO("auction_id: %d       local_id: %d", msg.get()->auction_id, 10000*robot_name + auction_id_number);
                bool robot_already_answered = false;

                for(unsigned int i = 0; i < robots_already_responded.size(); i++)
                {
                    if(robot_prefix_empty_param == true)
                    {
                        if(msg.get()->robot_name.compare(robots_already_responded.at(i).robot_str) == 0 && msg.get()->auction_id == robots_already_responded.at(i).auction_number)
                        {
                            ROS_WARN("Same msg already received!!!");
                            robot_already_answered = true;
                            break;
                        }
                    }else
                    {
                        ROS_INFO("Compare msg name: %d  responded: %d       msg auction: %d   responded: %d", robots_int_name, robots_already_responded.at(i).robot_number, msg.get()->auction_id, robots_already_responded.at(i).auction_number);
                        if(robots_int_name == robots_already_responded.at(i).robot_number && msg.get()->auction_id == robots_already_responded.at(i).auction_number)
                        {
                            ROS_WARN("Same msg already received!!!");
                            robot_already_answered = true;
                            break;
                        }
                    }
                }

                /*
                 * Only proceed if the robot has not answered before.
                 */
                if(robot_already_answered == false)
                {
                    if(robot_prefix_empty_param == true)
                    {
                        ROS_INFO("Auction from robot %s received", msg.get()->robot_name.c_str());
                    }else
                    {
                        ROS_INFO("Auction from robot %d received", robot_name);
                    }
                    auction_pair_t auction_pair;
                    auction_element_t auction_elements;

                    int has_cluster_id = -1;

                    /*
                     * Visualize the received message
                     */
                    for(unsigned int i = 0; i < msg.get()->available_clusters.size(); i++)
                    {
                        adhoc_communication::ExpCluster cluster_req;
                        cluster_req = msg.get()->available_clusters.at(i);
                        ROS_INFO("---------------------- %d ----------------------------", i);
                        std::string requested_clusters;
                        for(unsigned int j = 0; j < cluster_req.ids_contained.size(); j++)
                        {
                            if(j >= 6)
                            {
                                break;
                            }
                            requested_clusters.append(NumberToString((int)cluster_req.ids_contained.at(j).id));
                            requested_clusters.append(", ");
                        }
                        ROS_INFO("Received ids: %s", requested_clusters.c_str());
                    }



                    for(unsigned int i = 0; i < msg.get()->available_clusters.size(); i++)
                    {
                        adhoc_communication::ExpCluster current_cluster;
                        current_cluster = msg.get()->available_clusters.at(i);
                //        ROS_INFO("------------------------------------------------------------------");
                //        for(int k = 0; k < current_cluster.ids_contained.size(); k++)
                //        {
                //            ROS_INFO("FRONTIER ID: %d", current_cluster.ids_contained.at(k));
                //        }
                        int current_cluster_id = checkClustersID(current_cluster);

                        ROS_INFO("Received ID converted to cluster ID: %d", current_cluster_id);
                        if(current_cluster_id >= 0)
                        {
                            auction_pair.cluster_id = current_cluster_id;
                            auction_pair.bid_value = current_cluster.bid;
                            auction_elements.auction_element.push_back(auction_pair);
                            ROS_INFO("BID: %f",current_cluster.bid);
                        }
                    }
                //    ROS_INFO("Robot %d received all bids for all clusters", robot_name);
                    auction_elements.robot_id = robots_int_name;
                    auction_elements.detected_by_robot_str = msg.get()->robot_name;

                    auction_mutex.lock();
                    auction.push_back(auction_elements);
                    auction_mutex.unlock();

                    /*
                     * A BID is received from one robot. Remember the robot who send the
                     * bid for a special auction id!
                     */
    //                if(msg.get()->auction_id == 10000*robot_name + auction_id_number)
    //                {
                        number_of_auction_bids_received++;
                        responded_t auction_response;
                        auction_response.auction_number = msg.get()->auction_id;
                        auction_response.robot_number = robots_int_name;
                        if(robot_prefix_empty_param == true)
                        {
                            auction_response.robot_str = msg.get()->robot_name;
                        }
                        robots_already_responded.push_back(auction_response);
    //                }
                }else
                {
                    ROS_WARN("Robot already answered on this auction");
                }
            }
        }
    }
    auction_running = false;
}

int ExplorationPlanner::calculateAuctionBID(int cluster_number, std::string strategy)
{
//    ROS_INFO("Robot %d  calculates bid for cluster_number %d", robot_name, cluster_number);
    int auction_bid = 0;
    int cluster_vector_position = -1;
    bool cluster_could_be_found = false;

    if(clusters.size() > 0)
    {
        for (int i = 0; i < clusters.size(); i++)
        {
            if(clusters.at(i).id == cluster_number)
            {
//                if(clusters.at(i).cluster_element.size() > 0)
//                {
                    cluster_vector_position = i;
                    cluster_could_be_found = true;
                    break;
//                }else
//                {
//                    ROS_ERROR("Cluster found but empty, therefore do not calculate LOCAL BID");
//                    return(-1);
//                }
            }
        }
    }
    if(cluster_could_be_found == false)
    {
        /*
         * Cluster could not be found, set it to a high value like 100
         */
        ROS_WARN("Cluster could not be found");
        return(-1);
    }

    if (!costmap_global_ros_->getRobotPose(robotPose))
    {
            ROS_ERROR("Failed to get RobotPose");
    }

    int distance = -1;
    for(int i = 0; i < clusters.at(cluster_vector_position).cluster_element.size(); i++)
    {

        if(strategy == "trajectory")
        {
            distance = trajectory_plan(clusters.at(cluster_vector_position).cluster_element.at(i).x_coordinate, clusters.at(cluster_vector_position).cluster_element.at(i).y_coordinate);

        }else if(strategy == "euclidean")
        {
            double x = clusters.at(cluster_vector_position).cluster_element.at(i).x_coordinate - robotPose.getOrigin().getX();
            double y = clusters.at(cluster_vector_position).cluster_element.at(i).y_coordinate - robotPose.getOrigin().getY();
            distance = x * x + y * y;
            return distance;
        }

        if(distance > 0)
        {
            double x = clusters.at(cluster_vector_position).cluster_element.at(i).x_coordinate - robotPose.getOrigin().getX();
            double y = clusters.at(cluster_vector_position).cluster_element.at(i).y_coordinate - robotPose.getOrigin().getY();
            double euclidean_distance = x * x + y * y;

            /*
             * Check if the distance calculation is plausible.
             * The euclidean distance to this point need to be smaller then
             * the simulated trajectory path. If this stattement is not valid
             * the trajectory calculation has failed.
             */
//            ROS_INFO("Euclidean distance: %f   trajectory_path: %f", sqrt(euclidean_distance), distance* costmap_ros_->getCostmap()->getResolution());
            if (distance * costmap_ros_->getCostmap()->getResolution() <= sqrt(euclidean_distance)*0.95)
            {
                ROS_WARN("Euclidean distance smaller then trajectory distance to LOCAL CLUSTER!!!");
//                return(-1);
            }else
            {
                return distance;
            }
        }
    }
    if(distance == -1)
    {
//        ROS_ERROR("Unable to calculate LOCAL BID at position %d  --> BID: %d", cluster_vector_position, distance);
    }
//    ROS_INFO("Cluster at position %d  --> BID: %d", cluster_vector_position, distance);
    return(-1);
}

bool ExplorationPlanner::respondToAuction(std::vector<requested_cluster_t> requested_cluster_ids, int auction_id_number)
{
//    ros::Rate r(1);
//    while(ros::ok())
//    {
//        r.sleep();
////        while(start_thr_auction == false);
//        if(start_thr_auction == true)
//        {
            ROS_INFO("Respond to auction");
            adhoc_communication::ExpAuction auction_msgs;

            for(int i = 0; i < requested_cluster_ids.size(); i++)
            {
                ROS_INFO("Responding to cluster ids: %d", requested_cluster_ids.at(i).own_cluster_id);
            }

            for(int n = 0; n < requested_cluster_ids.size(); n++)
            {
                cluster_mutex.lock();
                for(int i = 0; i < clusters.size(); i++)
                {
                    if(clusters.at(i).id == requested_cluster_ids.at(n).own_cluster_id)
                    {
                        adhoc_communication::ExpCluster cluster_msg;
                        adhoc_communication::ExpClusterElement cluster_element_msg;
                        for(int j = 0; j < requested_cluster_ids.at(n).requested_ids.size(); j++)
                        {
                            if(robot_prefix_empty_param == true)
                            {
                                cluster_element_msg.id = requested_cluster_ids.at(n).requested_ids.at(j).id;
                                cluster_element_msg.detected_by_robot_str = requested_cluster_ids.at(n).requested_ids.at(j).robot_str;
                            }else
                            {
                                cluster_element_msg.id = requested_cluster_ids.at(n).requested_ids.at(j).id;
                            }
                            cluster_msg.ids_contained.push_back(cluster_element_msg);
                        }
//                        ROS_INFO("Calculate the auction BID");
                        cluster_msg.bid = calculateAuctionBID(clusters.at(i).id, trajectory_strategy);
                        auction_msgs.available_clusters.push_back(cluster_msg);
                        break;
                    }
                }
                cluster_mutex.unlock();
            }

            /*
             * Visualize the auction message to send
             */
//            for(int i = 0; i < auction_msgs.available_clusters.size(); i++)
//            {
//                adhoc_communication::Cluster cluster_msg_check;
//                cluster_msg_check = auction_msgs.available_clusters.at(i);
//                ROS_INFO("Robot %d sending BID: %f cluster elements: %u", robot_name, cluster_msg_check.bid, cluster_msg_check.ids_contained.size());
//            }

            ROS_INFO("Robot %d publishes auction bids for all clusters", robot_name);

//            /* FIXME */
//            if(auction_msgs.available_clusters.size() == 0)
//            {
//                adhoc_communication::Cluster cluster_msg;
//                cluster_msg.bid = -1;
//                auction_msgs.available_clusters.push_back(cluster_msg);
//            }
//

            auction_msgs.auction_status_message = false;
            auction_msgs.auction_id = auction_id_number;

            std::stringstream ss;
            ss << robot_name;
            std::string prefix = "";
            std::string robo_name = prefix.append(ss.str());

            auction_msgs.robot_name = robo_name;

//            if(first_run == true)
//            {
//                pub_auctioning_first.publish(auction_msgs);
//            }else
//            {
                sendToMulticastAuction("mc_", auction_msgs, "auction");
//            }

            start_thr_auction = false;
//        }
//    }
}

void ExplorationPlanner::negotiationCallback(const adhoc_communication::ExpFrontier::ConstPtr& msg)
{
//        ROS_ERROR("Negotiation List!!!");

        bool entry_found = false;

        adhoc_communication::ExpFrontierElement frontier_element;
        for(int j = 0; j < msg.get()->frontier_element.size(); j++)
        {
            frontier_element = msg.get()->frontier_element.at(j);
            for(int i = 0; i < negotiation_list.size(); i++)
            {
                if(robot_prefix_empty_param == true)
                {
                    if(negotiation_list.at(i).id == frontier_element.id && negotiation_list.at(i).detected_by_robot_str.compare(frontier_element.detected_by_robot_str) == 0)
                    {
                        entry_found = true;
                    }
                }else
                {
                    if(negotiation_list.at(i).id == frontier_element.id)
                    {
                        entry_found = true;
                    }
                }

            }
            if(entry_found == false)
            {
                ROS_DEBUG("Negotiation frontier with ID: %d", frontier_element.id);
                frontier_t negotiation_frontier;
                negotiation_frontier.detected_by_robot = frontier_element.detected_by_robot;
                negotiation_frontier.id = frontier_element.id;
                negotiation_frontier.x_coordinate = frontier_element.x_coordinate;
                negotiation_frontier.y_coordinate = frontier_element.y_coordinate;

                store_negotiation_mutex.lock();
                negotiation_list.push_back(negotiation_frontier);
                store_negotiation_mutex.unlock();
            }
        }
}


    pub_clusters = nh_cluster.advertise<geometry_msgs::PolygonStamped>("clusters", 2000, true);

    pub_cluster_grid_0 = nh_cluster_grid.advertise<nav_msgs::GridCells>("cluster_grid_0", 2000, true);
    pub_cluster_grid_1 = nh_cluster_grid.advertise<nav_msgs::GridCells>("cluster_grid_1", 2000, true);
    pub_cluster_grid_2 = nh_cluster_grid.advertise<nav_msgs::GridCells>("cluster_grid_2", 2000, true);
    pub_cluster_grid_3 = nh_cluster_grid.advertise<nav_msgs::GridCells>("cluster_grid_3", 2000, true);
    pub_cluster_grid_4 = nh_cluster_grid.advertise<nav_msgs::GridCells>("cluster_grid_4", 2000, true);
    pub_cluster_grid_5 = nh_cluster_grid.advertise<nav_msgs::GridCells>("cluster_grid_5", 2000, true);
    pub_cluster_grid_6 = nh_cluster_grid.advertise<nav_msgs::GridCells>("cluster_grid_6", 2000, true);
    pub_cluster_grid_7 = nh_cluster_grid.advertise<nav_msgs::GridCells>("cluster_grid_7", 2000, true);
    pub_cluster_grid_8 = nh_cluster_grid.advertise<nav_msgs::GridCells>("cluster_grid_8", 2000, true);
    pub_cluster_grid_9 = nh_cluster_grid.advertise<nav_msgs::GridCells>("cluster_grid_9", 2000, true);
    pub_cluster_grid_10 = nh_cluster_grid.advertise<nav_msgs::GridCells>("cluster_grid_10", 2000, true);
    pub_cluster_grid_11 = nh_cluster_grid.advertise<nav_msgs::GridCells>("cluster_grid_11", 2000, true);
    pub_cluster_grid_12 = nh_cluster_grid.advertise<nav_msgs::GridCells>("cluster_grid_12", 2000, true);
    pub_cluster_grid_13 = nh_cluster_grid.advertise<nav_msgs::GridCells>("cluster_grid_13", 2000, true);
    pub_cluster_grid_14 = nh_cluster_grid.advertise<nav_msgs::GridCells>("cluster_grid_14", 2000, true);
    pub_cluster_grid_15 = nh_cluster_grid.advertise<nav_msgs::GridCells>("cluster_grid_15", 2000, true);
    pub_cluster_grid_16 = nh_cluster_grid.advertise<nav_msgs::GridCells>("cluster_grid_16", 2000, true);
    pub_cluster_grid_17 = nh_cluster_grid.advertise<nav_msgs::GridCells>("cluster_grid_17", 2000, true);
    pub_cluster_grid_18 = nh_cluster_grid.advertise<nav_msgs::GridCells>("cluster_grid_18", 2000, true);
    pub_cluster_grid_19 = nh_cluster_grid.advertise<nav_msgs::GridCells>("cluster_grid_19", 2000, true);


void ExplorationPlanner::Callbacks()
{
    ros::Rate r(10);
    while(ros::ok())
    {
//        publish_subscribe_mutex.lock();

        if(robot_name == 1) //TODO
        {
                sub_frontiers = nh_frontier.subscribe("/robot_0/frontiers", 10000, &ExplorationPlanner::frontierCallback, this);
                sub_visited_frontiers = nh_visited_frontier.subscribe("/robot_0/visited_frontiers", 10000, &ExplorationPlanner::visited_frontierCallback, this);
                sub_negotioation = nh_negotiation.subscribe("/robot_0/negotiation_list", 10000, &ExplorationPlanner::negotiationCallback, this);
                sub_auctioning = nh_auction.subscribe("/robot_1/auction", 1000, &ExplorationPlanner::auctionCallback, this);

//                sub_frontiers = nh_frontier.subscribe("/robot_2/frontiers", 10000, &ExplorationPlanner::frontierCallback, this);
//                sub_visited_frontiers = nh_visited_frontier.subscribe("/robot_2/visited_frontiers", 10000, &ExplorationPlanner::visited_frontierCallback, this);
//                sub_negotioation = nh_negotiation.subscribe("/robot_2/negotiation_list", 10000, &ExplorationPlanner::negotiationCallback, this);
//
//                sub_frontiers = nh_frontier.subscribe("/robot_3/frontiers", 10000, &ExplorationPlanner::frontierCallback, this);
//                sub_visited_frontiers = nh_visited_frontier.subscribe("/robot_3/visited_frontiers", 10000, &ExplorationPlanner::visited_frontierCallback, this);
//                sub_negotioation = nh_negotiation.subscribe("/robot_3/negotiation_list", 10000, &ExplorationPlanner::negotiationCallback, this);

        }
        else if(robot_name == 0)
        {
                sub_frontiers = nh_frontier.subscribe("/robot_1/frontiers", 10000, &ExplorationPlanner::frontierCallback, this);
                sub_visited_frontiers = nh_visited_frontier.subscribe("/robot_1/visited_frontiers", 10000, &ExplorationPlanner::visited_frontierCallback, this);
                sub_negotioation = nh_negotiation.subscribe("/robot_1/negotiation_list", 10000, &ExplorationPlanner::negotiationCallback, this);
                sub_auctioning = nh_auction.subscribe("/robot_0/auction", 1000, &ExplorationPlanner::auctionCallback, this);
//                sub_frontiers = nh_frontier.subscribe("/robot_2/frontiers", 10000, &ExplorationPlanner::frontierCallback, this);
//                sub_visited_frontiers = nh_visited_frontier.subscribe("/robot_2/visited_frontiers", 10000, &ExplorationPlanner::visited_frontierCallback, this);
//                sub_negotioation = nh_negotiation.subscribe("/robot_2/negotiation_list", 10000, &ExplorationPlanner::negotiationCallback, this);
//
//                sub_frontiers = nh_frontier.subscribe("/robot_3/frontiers", 10000, &ExplorationPlanner::frontierCallback, this);
//                sub_visited_frontiers = nh_visited_frontier.subscribe("/robot_3/visited_frontiers", 10000, &ExplorationPlanner::visited_frontierCallback, this);
//                sub_negotioation = nh_negotiation.subscribe("/robot_3/negotiation_list", 10000, &ExplorationPlanner::negotiationCallback, this);

        }
//        publish_subscribe_mutex.unlock();

        r.sleep();
//        ros::spinOnce();
    }
}

//void ExplorationPlanner::initialize_planner(std::string name,
//		costmap_2d::Costmap2DROS *costmap, costmap_2d::Costmap2DROS *costmap_global) {

//    ROS_INFO("Initializing the planner");

//	//copy the pointed costmap to be available in ExplorationPlanner

//	this->costmap_ros_ = costmap;
//        this->costmap_global_ros_ = costmap_global;

//        if(initialized_planner == false)
//        {
//                nav.initialize("navigation_path", costmap_global_ros_);
//                initialized_planner = true;
//        }
//    int dim = costmap_global_ros_->getCostmap()->getSizeInCellsY() * costmap_global_ros_->getCostmap()->getSizeInCellsY();
//    float dim_meters = costmap_global_ros_->getCostmap()->getSizeInMetersY() * costmap_global_ros_->getCostmap()->getSizeInMetersY();
//    //ROS_ERROR("%d, %.0f, %.0f", dim, dim_meters, (float) dim * 0.05 * 0.05 );
//	//Occupancy_grid_array is updated here
//	this->setupMapData();

//	last_mode_ = FRONTIER_EXPLORE;
//	this->initialized_ = true;
//	
//	//ROS_ERROR("Initialized");

//	/*
//	 * reset all counter variables, used to count the number of according blocks
//	 * within the occupancy grid.
//	 */
//	unknown = 0, free = 0, lethal = 0, inflated = 0;

//}

void ExplorationPlanner::clusterFrontiers()
{

//    ROS_INFO("Clustering frontiers");

    int strategy = 1;
    /*
     * Strategy:
     * 1 ... merge clusters close together
     * 2 ... merge clusters based on model
     */

    bool cluster_found_flag = false, same_id = false;
    
    //store_frontier_mutex.lock();
    acquire_mutex(&store_frontier_mutex, __FUNCTION__);

    for(unsigned int i = 0; i < frontiers.size(); i++)
    {
        ROS_DEBUG("Frontier at: %u   and cluster size: %u",i, clusters.size());
        
        //F
        /* If the frontier has been already inserted in a cluster, move to next frontier */
        if(frontiers.at(i).cluster_id >= 0)
            continue;
        
        cluster_found_flag = false;
        bool frontier_used = false;
        same_id = false;

        for(unsigned int j = 0; j < clusters.size(); j++)
        {
            ROS_DEBUG("cluster %u contains %u elements", j, clusters.at(j).cluster_element.size());
            for(unsigned int n = 0; n < clusters.at(j).cluster_element.size(); n++)
            {
               ROS_DEBUG("accessing cluster %u   and element: %d", j, n);

               if(fabs(frontiers.at(i).x_coordinate - clusters.at(j).cluster_element.at(n).x_coordinate) < MAX_NEIGHBOR_DIST && fabs(frontiers.at(i).y_coordinate - clusters.at(j).cluster_element.at(n).y_coordinate) < MAX_NEIGHBOR_DIST)
               {
                   for(unsigned int m = 0; m < clusters.at(j).cluster_element.size(); m++)
                   {
                      ROS_DEBUG("checking id %d with element id: %d",frontiers.at(i).id,clusters.at(j).cluster_element.at(m).id);

                      if(robot_prefix_empty_param == true)
                      {
                          if(frontiers.at(i).id == clusters.at(j).cluster_element.at(m).id && frontiers.at(i).detected_by_robot_str.compare(clusters.at(j).cluster_element.at(m).detected_by_robot_str) == 0)
                          {
                              ROS_DEBUG("SAME ID FOUND !!!!!!!");
                              frontier_used = true;
                              same_id = true;
                              break;
                          }
                      }else
                      {
                          if(frontiers.at(i).id == clusters.at(j).cluster_element.at(m).id)
                          {
                              ROS_DEBUG("SAME ID FOUND !!!!!!!");
                              frontier_used = true;
                              same_id = true;
                              break;
                          }
                      }


                   }
                   if(same_id == false)
                   {
                      cluster_found_flag = true;
                   }
               }
               if(same_id == true || cluster_found_flag == true)
               {
                   break;
               }
            }
            if(same_id == true)
            {
                break;
            }else
            {
                if(cluster_found_flag == true)
                {
                    ROS_DEBUG("Frontier: %d attached", frontiers.at(i).id);
                    clusters.at(j).cluster_element.push_back(frontiers.at(i));
                    
                    //F
                    frontiers.at(i).cluster_id = clusters.at(j).id;
                    
                    frontier_used = true;
                    break;
                }
            }
        }
        if(cluster_found_flag == false && same_id == false)
        {
            ROS_DEBUG("ADD CLUSTER");
            cluster_t cluster_new;
            cluster_new.cluster_element.push_back(frontiers.at(i));
            cluster_new.id = (robot_name * 10000) + cluster_id++;
            
            //F
            frontiers.at(i).cluster_id = cluster_new.id;

            cluster_mutex.lock();
            clusters.push_back(cluster_new);
            cluster_mutex.unlock();

            ROS_DEBUG("Frontier: %d in new cluster", frontiers.at(i).id);
            frontier_used = true;
        }
        if(frontier_used == false)
        {
            ROS_WARN("Frontier: %d not used", frontiers.at(i).id);
        }
    }

    release_mutex(&store_frontier_mutex, __FUNCTION__);

    /*
     * To finish the process finally check whether a cluster is close to another one.
     * If so, merge them.
     */
    bool run_without_merging = false;

    if(strategy == 1)
    {
        while(run_without_merging == false)
        {
            bool merge_clusters = false;
            for(unsigned int i = 0; i < clusters.size(); i++)
            {
                for(unsigned int m = 0; m < clusters.at(i).cluster_element.size(); m ++)
                {
                    if(clusters.at(i).cluster_element.size() > 1)
                    {
                        for(unsigned int j = 0; j < clusters.size(); j++)
                        {
                            if(clusters.at(i).id != clusters.at(j).id)
                            {
                                if(clusters.at(j).cluster_element.size() > 1)
                                {
                                    for(unsigned int n = 0; n < clusters.at(j).cluster_element.size(); n++)
                                    {
                                        if(fabs(clusters.at(i).cluster_element.at(m).x_coordinate - clusters.at(j).cluster_element.at(n).x_coordinate) < CLUSTER_MERGING_DIST && fabs(clusters.at(i).cluster_element.at(m).y_coordinate - clusters.at(j).cluster_element.at(n).y_coordinate) < CLUSTER_MERGING_DIST)
                                        {
    //                                        ROS_INFO("Merge cluster %d", clusters.at(j).id);
                                            merge_clusters = true;
                                            break;
                                        }
                                    }
                                    if(merge_clusters == true)
                                    {
                                        /*
                                         * Now merge cluster i with cluster j.
                                         * afterwards delete cluster j.
                                         */
                                        for(unsigned int n = 0; n < clusters.at(j).cluster_element.size(); n++)
                                        {
                                            frontier_t frontier_to_merge;
                                            frontier_to_merge = clusters.at(j).cluster_element.at(n);
                                            clusters.at(i).cluster_element.push_back(frontier_to_merge);
                                            
                                            //F
                                            frontier_to_merge.cluster_id = clusters.at(i).id;
                                            
                                        }
    //                                    ROS_INFO("Erasing cluster %d", clusters.at(j).id);
                                        clusters.erase(clusters.begin() + j);
                                        break;
                                    }
                                }
                            }
                        }
                    }
                    if(merge_clusters == true)
                        break;
                }
                if(merge_clusters == true)
                    break;
            }
            if(merge_clusters == false)
            {
                run_without_merging = true;
            }
    //        ROS_INFO("RUN WITHOUT MERGING: %d", run_without_merging);
        }
    }
    else if(strategy == 2)
    {
        int MAX_NEIGHBOURS = 4;
        double costmap_resolution = costmap_ros_->getCostmap()->getResolution();

        while(run_without_merging == false)
        {
            bool merge_clusters = false;
            for(unsigned int i = 0; i < clusters.size(); i++)
            {
                for(unsigned int m = 0; m < clusters.at(i).cluster_element.size(); m ++)
                {
                    if(clusters.at(i).cluster_element.size() > 1)
                    {
                        for(unsigned int j = 0; j < clusters.size(); j++)
                        {
                            if(clusters.at(i).id != clusters.at(j).id)
                            {
                                if(clusters.at(j).cluster_element.size() > 1)
                                {
                                    for(unsigned int n = 0; n < clusters.at(j).cluster_element.size(); n++)
                                    {
                                        unsigned char cost;
                                        unsigned int mx,my;
                                        int freespace_detected = 0;
                                        int obstacle_detected  = 0;
                                        int elements_detected = 0;

                                        ROS_INFO("Calculating ...");
                                        int x_length = abs(clusters.at(i).cluster_element.at(m).x_coordinate - clusters.at(j).cluster_element.at(n).x_coordinate);
                                        int y_length = abs(clusters.at(i).cluster_element.at(m).y_coordinate - clusters.at(j).cluster_element.at(n).y_coordinate);

                                        ROS_INFO("x_length: %d    y_length: %d   resolution: %f", x_length, y_length,costmap_resolution);
                                        int x_elements = abs(x_length / costmap_resolution);
                                        int y_elements = abs(y_length / costmap_resolution);

                                        ROS_INFO("alpha");
                                        double alpha;
                                        if(x_length == 0)
                                        {
                                            alpha = 0;
                                        }
                                        else
                                        {
                                            alpha = atan(y_length/x_length);
                                        }

                                        ROS_INFO("atan: %f   x_length: %d    y_length: %d", alpha, x_length, y_length);

                                        for(int x = 0; x <= x_elements; x++)
                                        {
                                            /* increase neighbor size*/
                                            for(int neighbour = 0; neighbour < MAX_NEIGHBOURS; neighbour++)
                                            {
                                                int y = tan(alpha) * x * costmap_ros_->getCostmap()->getResolution();
                                                ROS_INFO("x: %d    y: %d", x, y);
                                                if(!costmap_global_ros_->getCostmap()->worldToMap(clusters.at(i).cluster_element.at(m).x_coordinate + x, clusters.at(i).cluster_element.at(m).y_coordinate + y + neighbour,mx,my))
                                                {
                                                    ROS_ERROR("Cannot convert coordinates successfully.");
                                                    continue;
                                                }
                                                
                                                //F
                                                //cost = costmap_global_ros_->getCostmap()->getCost(mx, my);
                                                cost = getCost(costmap_global_ros_, mx, my);

                                                if(cost == costmap_2d::FREE_SPACE)
                                                {
                                                    freespace_detected = true;
                                                }
                                                else if(cost == costmap_2d::LETHAL_OBSTACLE)
                                                {
                                                    obstacle_detected = true;
                                                }
                                                elements_detected++;
                                            }

                                            /* decrease neighbor size*/
                                            for(int neighbour = 0; neighbour < MAX_NEIGHBOURS; neighbour++)
                                            {
                                                int y = tan(alpha) * x * costmap_ros_->getCostmap()->getResolution();
                                                ROS_INFO("x: %d    y: %d", x, y);
                                                if(!costmap_global_ros_->getCostmap()->worldToMap(clusters.at(i).cluster_element.at(m).x_coordinate + x, clusters.at(i).cluster_element.at(m).y_coordinate + y - neighbour,mx,my))
                                                {
                                                    ROS_ERROR("Cannot convert coordinates successfully.");
                                                    continue;
                                                }
                                                
                                                //F
                                                //cost = costmap_global_ros_->getCostmap()->getCost(mx, my);
                                                cost = getCost(costmap_global_ros_, mx, my);

                                                if(cost == costmap_2d::FREE_SPACE)
                                                {
                                                    freespace_detected = true;
                                                }
                                                else if(cost == costmap_2d::LETHAL_OBSTACLE)
                                                {
                                                    obstacle_detected = true;
                                                }
                                                elements_detected++;
                                            }
                                        }


                                        ROS_INFO("*******************************");
                                        ROS_INFO("elements: %d    obstacle: %d", elements_detected, obstacle_detected);
                                        ROS_INFO("*******************************");
                                        if(elements_detected * 0.25 < obstacle_detected)
                                        {
                                            ROS_INFO("Merging clusters");
                                            merge_clusters = true;
                                            break;
                                        }
                                        break;
//                                        if(fabs(clusters.at(i).cluster_element.at(m).x_coordinate - clusters.at(j).cluster_element.at(n).x_coordinate) < CLUSTER_MERGING_DIST && fabs(clusters.at(i).cluster_element.at(m).y_coordinate - clusters.at(j).cluster_element.at(n).y_coordinate) < CLUSTER_MERGING_DIST)
//                                        {
//    //                                        ROS_INFO("Merge cluster %d", clusters.at(j).id);
//                                            merge_clusters = true;
//                                            break;
//                                        }
                                    }
                                    if(merge_clusters == true)
                                    {
                                        ROS_INFO("Merging");
                                        /*
                                         * Now merge cluster i with cluster j.
                                         * afterwards delete cluster j.
                                         */
                                        for(int n = 0; n < clusters.at(j).cluster_element.size(); n++)
                                        {
                                            frontier_t frontier_to_merge;
                                            frontier_to_merge = clusters.at(j).cluster_element.at(n);
                                            clusters.at(i).cluster_element.push_back(frontier_to_merge);
                                            
                                            //F
                                            frontier_to_merge.cluster_id = clusters.at(i).id;
                                            
                                        }
    //                                    ROS_INFO("Erasing cluster %d", clusters.at(j).id);
                                        clusters.erase(clusters.begin() + j);
                                        j--;
                                        ROS_INFO("Done merging");
                                        break;
                                    }
                                }
                            }
                            break;
                        }
                    }
                    if(merge_clusters == true)
                        break;
                }
                if(merge_clusters == true)
                    break;
            }
            if(merge_clusters == false)
            {
                run_without_merging = true;
            }
    //        ROS_INFO("RUN WITHOUT MERGING: %d", run_without_merging);
        }
    }
    
}

void ExplorationPlanner::visualizeClustersConsole()
{
    ROS_INFO("------------------------------------------------------------------");
    for(int j = 0; j < clusters.size(); j++)
    {
        for(int n = 0; n < clusters.at(j).cluster_element.size(); n++)
        {
            if(robot_prefix_empty_param == true)
            {
                ROS_INFO("ID: %6d  x: %5.2f  y: %5.2f  cluster: %5d   robot: %s", clusters.at(j).cluster_element.at(n).id, clusters.at(j).cluster_element.at(n).x_coordinate, clusters.at(j).cluster_element.at(n).y_coordinate, clusters.at(j).id, clusters.at(j).cluster_element.at(n).detected_by_robot_str.c_str());
            }else
            {
                ROS_INFO("ID: %6d  x: %5.2f  y: %5.2f  cluster: %5d   dist: %d", clusters.at(j).cluster_element.at(n).id, clusters.at(j).cluster_element.at(n).x_coordinate, clusters.at(j).cluster_element.at(n).y_coordinate, clusters.at(j).id, clusters.at(j).cluster_element.at(n).dist_to_robot);
            }
        }
    }
    ROS_INFO("------------------------------------------------------------------");
}


/**
 * Compute the length of the trajectory from the robots current position to the first ten frontiers in the frontiers array
 */
void ExplorationPlanner::trajectory_plan_10_frontiers()
{
    for(int i = 0; i<10 && i<frontiers.size(); i++)
    {
        frontiers.at(i).distance_to_robot = trajectory_plan(frontiers.at(i).x_coordinate, frontiers.at(i).y_coordinate);
    }
}


/**
 * Compute the length of the trajectory from the robots current position to a given target
 */
int ExplorationPlanner::trajectory_plan(double target_x, double target_y)
{
    if (!costmap_global_ros_->getRobotPose(robotPose))
    {
        ROS_ERROR("Failed to get RobotPose");
        return -1;
    }
    //ROS_ERROR("%f", costmap_global_ros_->getCostmap()->getResolution());
    return trajectory_plan(robotPose.getOrigin().getX(), robotPose.getOrigin().getY(), target_x, target_y);
}

/**
 * Compute the length of the trajectory from a given start to a given target in number of grid cells
 */
int ExplorationPlanner::trajectory_plan(double start_x, double start_y, double target_x, double target_y)
{
    ROS_FATAL("should not be called!");
    geometry_msgs::PoseStamped goalPointSimulated, startPointSimulated;
    int distance;

    std::vector<double> backoffGoal;
    bool backoff_flag = smartGoalBackoff(target_x,target_y, costmap_global_ros_, &backoffGoal);

    startPointSimulated.header.seq = start_point_simulated_message++;	// increase the sequence number
    startPointSimulated.header.stamp = ros::Time::now();
    startPointSimulated.header.frame_id = move_base_frame;
    startPointSimulated.pose.position.x = start_x;
    startPointSimulated.pose.position.y = start_y;
    startPointSimulated.pose.position.z = 0;
    startPointSimulated.pose.orientation.x = 0;
    startPointSimulated.pose.orientation.y = 0;
    startPointSimulated.pose.orientation.z = 0;
    startPointSimulated.pose.orientation.w = 1;

    goalPointSimulated.header.seq = goal_point_simulated_message++;	// increase the sequence number
    goalPointSimulated.header.stamp = ros::Time::now();
    goalPointSimulated.header.frame_id = move_base_frame;
    if(backoff_flag == true)
    {
        goalPointSimulated.pose.position.x = backoffGoal.at(0);
        goalPointSimulated.pose.position.y = backoffGoal.at(1);
    }
    else
    {
        goalPointSimulated.pose.position.x = target_x;
        goalPointSimulated.pose.position.y = target_y;
    }
    goalPointSimulated.pose.position.z = 0;
    goalPointSimulated.pose.orientation.x = 0;
    goalPointSimulated.pose.orientation.y = 0;
    goalPointSimulated.pose.orientation.z = 0;
    goalPointSimulated.pose.orientation.w = 1;

    std::vector<geometry_msgs::PoseStamped> global_plan;

    //acquire_mutex(&costmap_mutex, __FUNCTION__);
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(costmap_global_ros_->getCostmap()->getMutex()));
    bool successful = nav.makePlan(startPointSimulated, goalPointSimulated, global_plan);
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> unlock(*(costmap_global_ros_->getCostmap()->getMutex()));
    //release_mutex(&costmap_mutex, __FUNCTION__);    
    
    if(successful == true)
    {
        //ROS_ERROR("Path from (%f, %f) to (%f, %f)", startPointSimulated.pose.position.x, startPointSimulated.pose.position.y, goalPointSimulated.pose.position.x, goalPointSimulated.pose.position.y);
        distance =  global_plan.size();
        //for(int i=0; i < global_plan.size(); i++)
            //ROS_ERROR("(%f, %f)", global_plan[i].pose.position.x, global_plan[i].pose.position.y);
        
        /*
        distance = 0;
        std::vector<geometry_msgs::PoseStamped>::iterator it = global_plan.begin();
        geometry_msgs::PoseStamped prev_point = (*it);
        it++;
        for(; it != global_plan.end(); it++) {
            distance = sqrt( (prev_point.pose.position.x - (*it).pose.position.x) * (prev_point.pose.position.x - (*it).pose.position.x) + (prev_point.pose.position.y - (*it).pose.position.y) * (prev_point.pose.position.y - (*it).pose.position.y) );
            prev_point = (*it);
        }
        */
        
        
        return distance;
    }
    else
    {
        //ROS_ERROR("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        return -1;
    }
}

//double ExplorationPlanner::trajectory_plan_print(double start_x, double start_y, double target_x, double target_y)
//{
//    geometry_msgs::PoseStamped goalPointSimulated, startPointSimulated;
//    double distance;

//    std::vector<double> backoffGoal;
//    bool backoff_flag = smartGoalBackoff(target_x,target_y, costmap_global_ros_, &backoffGoal);

//    startPointSimulated.header.seq = start_point_simulated_message++;	// increase the sequence number
//    startPointSimulated.header.stamp = ros::Time::now();
//    startPointSimulated.header.frame_id = move_base_frame;
//    startPointSimulated.pose.position.x = start_x;
//    startPointSimulated.pose.position.y = start_y;
//    startPointSimulated.pose.position.z = 0;
//    startPointSimulated.pose.orientation.x = 0;
//    startPointSimulated.pose.orientation.y = 0;
//    startPointSimulated.pose.orientation.z = 0;
//    startPointSimulated.pose.orientation.w = 1;

//    goalPointSimulated.header.seq = goal_point_simulated_message++;	// increase the sequence number
//    goalPointSimulated.header.stamp = ros::Time::now();
//    goalPointSimulated.header.frame_id = move_base_frame;
//    if(backoff_flag == true)
//    {
//        goalPointSimulated.pose.position.x = backoffGoal.at(0);
//        goalPointSimulated.pose.position.y = backoffGoal.at(1);
//    }
//    else
//    {
//        goalPointSimulated.pose.position.x = target_x;
//        goalPointSimulated.pose.position.y = target_y;
//    }
//    goalPointSimulated.pose.position.z = 0;
//    goalPointSimulated.pose.orientation.x = 0;
//    goalPointSimulated.pose.orientation.y = 0;
//    goalPointSimulated.pose.orientation.z = 0;
//    goalPointSimulated.pose.orientation.w = 1;

//    std::vector<geometry_msgs::PoseStamped> global_plan;

//    //acquire_mutex(&costmap_mutex, __FUNCTION__);
//    bool successful = nav.makePlan(startPointSimulated, goalPointSimulated, global_plan);
//    //release_mutex(&costmap_mutex, __FUNCTION__);
//    
//    //ROS_ERROR("%d", successful);
//    //ros::Duration(2).sleep();
//    
//    if(successful == true)
//    {
//        //ROS_ERROR("Path from (%f, %f) to (%f, %f)", startPointSimulated.pose.position.x, startPointSimulated.pose.position.y, goalPointSimulated.pose.position.x, goalPointSimulated.pose.position.y);
//        //distance =  global_plan.size();
//        for(int i=0; i < global_plan.size(); i++)
//            ROS_ERROR("(%f, %f)", global_plan[i].pose.position.x, global_plan[i].pose.position.y);
//        
//        distance = 0;
//        std::vector<geometry_msgs::PoseStamped>::iterator it = global_plan.begin();
//        geometry_msgs::PoseStamped prev_point = (*it);
//        it++;
//        for(; it != global_plan.end(); it++) {
//            distance += sqrt( (prev_point.pose.position.x - (*it).pose.position.x) * (prev_point.pose.position.x - (*it).pose.position.x) + (prev_point.pose.position.y - (*it).pose.position.y) * (prev_point.pose.position.y - (*it).pose.position.y) ); //* costmap_global_ros_->getCostmap()->getResolution();
//            prev_point = (*it);
//        }
//        
//        //ROS_ERROR("%f", distance);
//        return distance;
//    }
//    else
//    {
//        //ROS_ERROR("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
//        return -1;
//    }
//}

bool ExplorationPlanner::negotiate_Frontier(double x, double y, int detected_by, int id, int cluster_id_number)
{
    ROS_INFO("Negotiating Frontier with id: %d  at Cluster: %d", id, cluster_id_number);

    int cluster_vector_position = 0;
    for (int i = 0; i < clusters.size(); i++)
    {
        if(clusters.at(i).id == cluster_id_number)
        {
            cluster_vector_position = i;
            break;
        }
    }

    ROS_DEBUG("cluster vector position: %d", cluster_vector_position);

    bool entry_found = false;
    bool id_in_actual_cluster = false;

    for(int i = 0; i< negotiation_list.size(); i++)
    {
        for(int k = 0; k < clusters.at(cluster_vector_position).cluster_element.size(); k++)
        {
            id_in_actual_cluster = false;
            if(negotiation_list.at(i).id == clusters.at(cluster_vector_position).cluster_element.at(k).id)
            {
                ROS_INFO("         Same ID detected");
                for(int j = 0; j < clusters.at(cluster_vector_position).cluster_element.size(); j++)
                {
                    for(int m = 0; m < my_negotiation_list.size(); m++)
                    {
                        if(clusters.at(cluster_vector_position).cluster_element.at(j).id == my_negotiation_list.at(m).id)
                        {
                            ROS_INFO("         But is in the current working cluster! everything alright");
                            id_in_actual_cluster = true;
                            return true;
                        }
                    }
                }

                double used_cluster_ids = 0;
                if(id_in_actual_cluster == false)
                {
                    ROS_INFO("Checking how many goals in the cluster are already occupied");
                    for(int n = 0; n < negotiation_list.size(); n++)
                    {
                        for(int m = 0; m < clusters.at(cluster_vector_position).cluster_element.size(); m++)
                        {
                            if(negotiation_list.at(n).id == clusters.at(cluster_vector_position).cluster_element.at(m).id)
                            {
                                used_cluster_ids++;
                            }
                        }
                    }
//                    ROS_INFO("%f goals of %f in the cluster  -> %f",used_cluster_ids, (double)clusters.at(cluster_vector_position).cluster_element.size(), double(used_cluster_ids / (double)clusters.at(cluster_vector_position).cluster_element.size()));
                    if(double(used_cluster_ids / (double)clusters.at(cluster_vector_position).cluster_element.size()) >= 0.1)
                    {
                        ROS_INFO("Negotiation failed the other Robot got cluster %d already", cluster_id_number);
                        entry_found = true;
                        return false;
                    }
                }
            }
        }
    }
    if(entry_found == false)
    {
        frontier_t new_frontier;
        new_frontier.x_coordinate = x;
        new_frontier.y_coordinate = y;
        new_frontier.detected_by_robot = detected_by;
        new_frontier.id = id;

        publish_negotiation_list(new_frontier, cluster_id_number);

        return true;
    }
    return false;
}

//bool ExplorationPlanner::removeUnreachableFrontier(int id, std::string detected_by_robot_str)
//{
//    for(int i= 0; i< unreachable_frontiers.size(); i++)
//    {
//        if(robot_prefix_empty_param == true)
//        {
//            if(unreachable_frontiers.at(i).id == id && unreachable_frontiers.at(i).detected_by_robot_str.compare(detected_by_robot_str) == 0)
//            {
//                ROS_INFO("Removing Unreachable Frontier ID: %d  at position: %d  of Robot: %s", unreachable_frontiers.at(i).id, i, unreachable_frontiers.at(i).detected_by_robot_str.c_str());

//                unreachable_frontiers.erase(unreachable_frontiers.begin()+i);
////                if(i > 0)
////                {
////                    i --;
////                }
//                break;
//            }
//        }else
//        {
//            if(unreachable_frontiers.at(i).id == id)
//            {
//                unreachable_frontiers.erase(unreachable_frontiers.begin()+i);
//                if(i > 0)
//                {
//                    i --;
//                }
//                break;
//            }
//        }
//    }

//    return true;
//}

bool ExplorationPlanner::publish_negotiation_list(frontier_t negotiation_frontier, int cluster_number)
{
//    ROS_ERROR("Publish negotiation list!!!!!!!!!!!!!!!!");
    adhoc_communication::ExpFrontier negotiation_msg;

//    for(int i = 0; i<negotiation_list.size(); i++)
//    {
//        adhoc_communication::FrontierElement negotiation_element;
//        negotiation_element.detected_by_robot = negotiation_list.at(i).detected_by_robot;
//        negotiation_element.x_coordinate = negotiation_list.at(i).x_coordinate;
//        negotiation_element.y_coordinate = negotiation_list.at(i).y_coordinate;
//        negotiation_element.id = negotiation_list.at(i).id;
//
//        negotiation_msg.frontier_element.push_back(negotiation_element);
////        pub_negotion.publish(negotiation_msg);
//        sendToMulticast("mc_",negotiation_msg, "negotiation_list");
//    }

    if(cluster_number != -1)
    {
        int cluster_vector_position = 0;
        for (int i = 0; i < clusters.size(); i++)
        {
            if(clusters.at(i).id == cluster_number)
            {
                ROS_DEBUG("Cluster ID: %d   is at vector position: %d", cluster_number, i);
                cluster_vector_position = i;
                break;
            }
        }

        for(int i = 0; i < clusters.at(cluster_vector_position).cluster_element.size(); i++)
        {
            adhoc_communication::ExpFrontierElement negotiation_element;
            negotiation_element.x_coordinate = clusters.at(cluster_vector_position).cluster_element.at(i).x_coordinate;
            negotiation_element.y_coordinate = clusters.at(cluster_vector_position).cluster_element.at(i).y_coordinate;
            negotiation_element.detected_by_robot = clusters.at(cluster_vector_position).cluster_element.at(i).detected_by_robot;
            negotiation_element.id = clusters.at(cluster_vector_position).cluster_element.at(i).id;

            frontier_t new_frontier;
            new_frontier.x_coordinate = clusters.at(cluster_vector_position).cluster_element.at(i).x_coordinate;
            new_frontier.y_coordinate = clusters.at(cluster_vector_position).cluster_element.at(i).y_coordinate;
            new_frontier.detected_by_robot = clusters.at(cluster_vector_position).cluster_element.at(i).detected_by_robot;
            new_frontier.id = clusters.at(cluster_vector_position).cluster_element.at(i).id;

            negotiation_msg.frontier_element.push_back(negotiation_element);
            my_negotiation_list.push_back(new_frontier);
        }
//        return true;
    }else
    {
        adhoc_communication::ExpFrontierElement negotiation_element;
        negotiation_element.detected_by_robot = negotiation_frontier.detected_by_robot;
        negotiation_element.x_coordinate = negotiation_frontier.x_coordinate;
        negotiation_element.y_coordinate = negotiation_frontier.y_coordinate;
        negotiation_element.id = negotiation_frontier.id;

        negotiation_msg.frontier_element.push_back(negotiation_element); //FIXME
    }

    sendToMulticast("mc_",negotiation_msg, "negotiation_list");

    first_negotiation_run = false;
}
