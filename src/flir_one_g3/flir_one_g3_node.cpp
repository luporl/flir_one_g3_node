#include "ros/ros.h"

#include "std_msgs/String.h"

#include <sstream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "flir_one_g3_node");

    ros::NodeHandle n;

    // (topic, msg_queue_size)
    ros::Publisher dbg_pub = n.advertise<std_msgs::String>("/f1g3/dbg", 1000);

    ros::Rate loop_rate(1);     // Hz

    for (int i = 0; ros::ok(); i++) {
        std_msgs::String msg;

        std::stringstream ss;
        ss << "f1g3_dbg#" << std::setw(3) << i;
        msg.data = ss.str();

        ROS_INFO("%s", msg.data.c_str());
        dbg_pub.publish(msg);

        /* process callbacks */
        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}
