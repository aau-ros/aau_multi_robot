struct bid_t
{
    unsigned int robot_id;
    double bid;
};

//TODO put this here or inside the class? where is better?
struct auction_t
{
    unsigned int auction_id;
    double starting_time;
    unsigned int auctioneer;
    unsigned int docking_station_id;
//    double ending_time;
    unsigned int winner_robot;
};

