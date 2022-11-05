#include <ros/ros.h>
#include <std_msgs/String.h>

void callback_function(const std_msgs::String &msg)
{
    ROS_INFO("received [%s]", msg.data.c_str());
}

int main(int argc, char** argv)
{
    std::string topicName="chat";
    std::string nodeName="listener";

    ros::init(argc, argv, nodeName );

    std::size_t queue_size=10;
    ros::NodeHandle nh=ros::NodeHandle();
    ros::Subscriber subscriber=nh.subscribe(topicName, queue_size,callback_function  );
    ros::spin();

}
