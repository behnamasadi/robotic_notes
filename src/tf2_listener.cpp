#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>

//rosrun tf tf_echo /world /child_frame_id

void callBackMessage(const geometry_msgs::TransformStamped  &msgs)
{
    //std::cout<<msg[0].child_frame_id <<std::endl;
}


int main(int argc, char** argv)
{
    std::string nodeName="tf2_listener";
    ros::init(argc, argv, nodeName,ros::init_options::AnonymousName );

    ros::NodeHandle nh=ros::NodeHandle("~my_ns");
   


    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tf2_listener(tfBuffer);

    while(ros::ok())
    {
        geometry_msgs::TransformStamped transformStamped;
        try
        {
            transformStamped = tfBuffer.lookupTransform("world","child_frame_id", ros::Time(0));
        }catch(tf2::TransformException &exception)
        {
            ROS_WARN("%s",exception.what());
            ros::Duration(1.0).sleep();
            continue;
        }
    }





    return 0;
}
