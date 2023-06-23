#if !defined(COMMUNICATOR_HPP)
#define COMMUNICATOR_HPP

#include "DataStreamClient.h"
#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <map>
#include <chrono>
#include <string>
#include <unistd.h>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

using namespace std;

// Struct used to hold segment data to transmit to the Publisher class.
struct PositionStruct
{
    double translation[3];
    double rotation[4];
    std::string subject_name;
    std::string segment_name;
    std::string translation_type;
    unsigned int frame_number;

} typedef PositionStruct;


// Main Node class
class Communicator : public rclcpp::Node
{
private:
    ViconDataStreamSDK::CPP::Client vicon_client;
    string hostname;
    unsigned int buffer_size;
    string ns_name;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    void publish_tf(PositionStruct p);

public:
    Communicator();

    // Initialises the connection to the DataStream server
    bool connect();

    // Stops the current connection to a DataStream server (if any).
    bool disconnect();

    // Main loop that request frames from the currently connected DataStream server and send the 
    // received segment data to the Publisher class.
    void get_frame();

    //// functions to create a segment publisher in a new thread
    //void create_publisher(const string subject_name, const string segment_name);
    //void create_publisher_thread(const string subject_name, const string segment_name);
};

#endif // COMMUNICATOR_HPP
