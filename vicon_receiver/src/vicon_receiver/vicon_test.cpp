
#include <iostream>
#include <unistd.h>
#include <chrono>

//#include <ViconDataStreamSDK_CPP/DataStreamClient.h>
#include "vicon_receiver/DataStreamClient.h"


// ros
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace ViconDataStreamSDK::CPP;



class ViconBridge : public rclcpp::Node 
{

	public:
	ViconBridge() ;
	~ViconBridge();


	
	private:
	bool init_vicon();
	bool setup_vicon();
	bool run_vicon();
	
	Client vicon_client_;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;



	std::chrono::time_point<std::chrono::high_resolution_clock> last_frame_time_;
	double maxT = 0.0;
	double avgT = 0.0;

};


ViconBridge::ViconBridge() : Node("vicon_bridge"){
	// starting rclcpp
	publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);

	std::cout << "starting init" << std::endl;

        init_vicon();

	std::cout << "starting setup" << std::endl;
	setup_vicon();


	std::cout << "starting...."<< std::endl;
	last_frame_time_ = std::chrono::high_resolution_clock::now();
	run_vicon();


}


ViconBridge::~ViconBridge() {

	vicon_client_.Disconnect();
	std::cout <<"client disconnected" << std::endl;

}



bool ViconBridge::init_vicon(){

	while (!vicon_client_.IsConnected().Connected)
    {
      std::string host_name_= "192.168.2.136:801";
      vicon_client_.Connect(host_name_);
      std::cout << ".\n";
      sleep(1);

    }
	std::cout << "successfully connected\n";
	return true;


}

bool ViconBridge::setup_vicon() {

	std::cout << "in setup" << std::endl;

	std::string mode = "ServerPush";

	if (mode == "ServerPush") {
	
	if (vicon_client_.SetStreamMode(StreamMode::ServerPush).Result) {
		std::cout << "mode set to client pull" << std::endl;
	}
	}

	if (mode == "ClientPull") {
	if (vicon_client_.SetStreamMode(StreamMode::ClientPull).Result) {
		std::cout << "mode set to client pull" << std::endl;
	}
	}

	//vicon_client_.SetAxisMapping(Direction::Forward, Direction::Left, Direction::Up);
	

	vicon_client_.EnableSegmentData();


	Output_GetVersion _Output_GetVersion = vicon_client_.GetVersion();
	std::cout << "Version: " << _Output_GetVersion.Major << "." << _Output_GetVersion.Minor << "."
        << _Output_GetVersion.Point << std::endl;


	return true;

}


bool ViconBridge::run_vicon() {


	while(vicon_client_.IsConnected().Connected ) {

		Output_GetFrame output = vicon_client_.GetFrame();
		auto now = std::chrono::high_resolution_clock::now();

		std::chrono::duration<double> diff = now - last_frame_time_;
		last_frame_time_ = now;

		if (maxT < diff.count()) {
			maxT = diff.count();
		}
		
		double f = 0.9;
		avgT = f * avgT + (1 - f) * diff.count();

		std::cout << "got frame. MAX: " << maxT << " AVG: " << avgT << " THIS: " << diff.count() << "s\n";
		auto message = std_msgs::msg::String();
	  	message.data = "frame";
	  	//RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
		publisher_->publish(message);



	}

	return true;

}


int main(int argc, char* argv[]) 
{

	std::cout << "hi" << std::endl;

	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ViconBridge>());
	rclcpp::shutdown();


	//ViconBridge bridge;
	//bridge.init_vicon();


	return 0;
}
