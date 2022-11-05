#include <ros/ros.h>
#include <std_msgs/String.h>

#include <ctime>
#include <signal.h>
   
void mySigintHandler(int sig)
{
    // Do some custom action.
    // For example, publish a stop message to some other nodes.

    // All the default sigint handler does is call shutdown()
    ros::shutdown();
}


int main(int argc, char **argv)
{

/*
node_name
will be assigned to your node unless it's overridden by one of the remapping arguments. 
names must be unique across the ROS system. If a second node is started with the same name as the first,
the first will be shutdown automatically. 
 
In cases where you want multiple of the same node, use the init_options::AnonymousName  to make the name unique.

*/

    std::string nodeName="talker";
    uint32_t options;


    //options=ros::init_options::AnonymousName;
    options=0;


    // It does not contact the master. 
    ros::init(argc, argv,nodeName,options );

/*
you might have multiple instance of a robot i.e multiple turtlebot and each of them runs
 all its nodes and topics in its own namespace. 
 This makes it easier to see 
1) which robot is running on, 
2) makes it possible to run several robot at the same time. 
this make sure so all nodes stick to the namespacing conventions.
*/
    

    //public: /namespace/topic
    //ros::NodeHandle nh=ros::NodeHandle();

    //private: /namespace/node/topic
    ros::NodeHandle nh=ros::NodeHandle("~my_namespace");

    //namespaced: /namespace/node/topic
    //ros::NodeHandle nh=ros::NodeHandle("my_namespace");

    //global: /topic
    //ros::NodeHandle nh=ros::NodeHandle("/");


    ros::Rate loopRate(0.5);
    std::string topicName="chat";
    ros::Publisher msgPublisher= nh.advertise<std_msgs::String>(topicName,1);


   



    std_msgs::String  msg;

    while (ros::ok())
    {
        std::time_t result = std::time(nullptr);
         
        msg.data=std::asctime(std::localtime(&result));

   

        msgPublisher.publish(msg );    
        ros::spinOnce();
        loopRate.sleep();
    }
    
    signal(SIGINT, mySigintHandler);
    

}
