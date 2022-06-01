#include "ros/ros.h"

#include "std_msgs/String.h"

#include "flirone.h"

#include <sstream>

int main(int argc, char **argv)
{
    struct f1_cfg f1cfg = {};
    int rc;

    f1cfg.pal_path = "palettes/Rainbow.raw";
    if (f1_init(&f1cfg) != 0)
        return -1;

    ros::init(argc, argv, "flir_one_g3_node");

    ros::NodeHandle n;

    // (topic, msg_queue_size)
    ros::Publisher dbg_pub = n.advertise<std_msgs::String>("/f1g3/dbg", 1000);

    for (int i = 0; ros::ok(); i++) {
        std_msgs::String msg;

        std::stringstream ss;
        ss << "f1g3_dbg#" << std::setw(3) << i;
        msg.data = ss.str();

        ROS_INFO("%s", msg.data.c_str());
        dbg_pub.publish(msg);

        rc = f1_loop();
        if (rc < 0) {
            ROS_INFO("f1_loop: error");
        } else {
            ROS_INFO("f1_loop: ok");
            if (rc & F1L_NEW_IMG_FRAME)
                ROS_INFO("f1_loop: NEW_IMG_FRAME");
            if (rc & F1L_NEW_IR_FRAME)
                ROS_INFO("f1_loop: NEW_IR_FRAME");
            if (rc & F1L_BUSY)
                ROS_INFO("f1_loop: BUSY");
        }

        /* process callbacks */
        ros::spinOnce();
    }

    return 0;
}
