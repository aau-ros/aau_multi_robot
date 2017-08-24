#ifndef FRONTIER_SELECTOR_H
#define FRONTIER_SELECTOR_H

#include <ros/ros.h>

struct frontier_t {
    unsigned int id;
    double x_coordinate;
    double y_coordinate;
    double cost;
    double detected_by_robot;
};

class FrontierSelector {
public:
    FrontierSelector();
    bool selectFrontier(double available_distance, std::vector<double> *final_goal, int count, std::vector<std::string> *robot_str_name, int actual_cluster_id, bool energy_above_th);

private:
    int w1, w2, w3, w4;
    unsigned int num_robots;
    std::vector<frontier_t> sorted_frontiers, frontiers_under_auction;
    frontier_t *my_selected_frontier, frontier_under_negotiation;
    double auction_timeout, my_bid;
    bool winner_of_auction;

    void loadParameters();
    template <typename T>
    void getParam(std::string param_name, T &param);
    void sendSelectedFrontier();
    void my_negotiate();
};

#endif // FRONTIER_SELECTOR_H
