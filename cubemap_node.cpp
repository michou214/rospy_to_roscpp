
#include "cubemap_node.h"
#include 

#include <opencv2/opencv.hpp>
using namespace cv;
#include <ros/ros.h>




class CubemapNode{ 
    public: 
	CubemapNode();

	struct NomDeVotreStructure

{

    int variable1;

    int variable2;

    int autreVariable;

    double nombreDecimal;

};

	// Initialize node
    ros::init(argc, argv, "image_node");
    ROS_INFO("Image node Initialized");

    ros::NodeHandle nh;

    type image_callback(){

    }
    type camera_callback(){

    }
    type publish_image(){

    }


	// Access specifier 
    private: 

}; 











































































