#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>


/*
rosrun tf2_ros static_transform_publisher x y z yaw pitch roll frame_id child_frame_id 

<launch>
<node pkg="tf2_ros" type="static_transform_publisher" name="link1_broadcaster" args="1 0 0 0 0 0 1 link1_parent link1" />
</launch>

http://wiki.ros.org/tf2_ros
*/


int main(int argc, char ** argv)
{
    std::string nodeName="tf_broadcaster";

    ros::init(argc,argv, nodeName, ros::init_options::AnonymousName);


    ros::NodeHandle nh=ros::NodeHandle("~my_ns");
    //nh.advertise<std_msgs::>
    ros::Rate loopRate(10);
    geometry_msgs::TransformStamped static_transformStamped;

    static_transformStamped.child_frame_id="child_frame_id";
    
    static_transformStamped.header.frame_id="world";
    static_transformStamped.header.stamp=ros::Time::now();
    //static_transformStamped.header.seq

    tf2::Quaternion quaternion;
    
    quaternion.setRPY(0,M_PI/2,0);

    static_transformStamped.transform.rotation.x=quaternion.x();
    static_transformStamped.transform.rotation.y=quaternion.y();
    static_transformStamped.transform.rotation.z=quaternion.z();
    static_transformStamped.transform.rotation.w=quaternion.w();

    static_transformStamped.transform.translation.x=1;
    static_transformStamped.transform.translation.y=-1;
    static_transformStamped.transform.translation.z=2;


    static tf2_ros::StaticTransformBroadcaster static_broadcaster;

    while (ros::ok())
    {
        static_broadcaster.sendTransform(static_transformStamped);
        loopRate.sleep();
    }
    

 


}