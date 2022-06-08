// Own
#include "flirone.h"

// ROS
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/CompressedImage.h>

// C++
#include <sstream>
#include <vector>

// C
#include <sys/time.h>

// MJPEG publisher
class Mjpeg {
public:
    Mjpeg(ros::Publisher &pub) : m_pub(pub)
    {}

    void publish()
    {
        struct timeval tv = {};
        gettimeofday(&tv, NULL);

        sensor_msgs::CompressedImage msg;
        msg.header.seq = m_seq++;
        msg.header.stamp.sec = tv.tv_sec;
        msg.header.stamp.nsec = tv.tv_usec * 1000;
        msg.header.frame_id = "optical_camera";
        msg.format = "jpeg";
        msg.data = std::vector<unsigned char>(f1_jpg_ptr,
            f1_jpg_ptr + f1_jpg_sz);
        m_pub.publish(msg);
    }

private:
    ros::Publisher &m_pub;
    uint32_t m_seq = 1;
};

int main(int argc, char **argv)
{
    struct f1_cfg f1cfg = {};
    int rc;

    // Init flirone
    f1cfg.pal_path = "palettes/Rainbow.raw";
    if (f1_init(&f1cfg) != 0)
        return -1;

    // Init ROS
    ros::init(argc, argv, "flir_one_g3_node");
    ros::NodeHandle n;

    // Setup publishers
    // (topic, msg_queue_size)
    ros::Publisher dbg_pub = n.advertise<std_msgs::String>("/f1g3/dbg", 1000);
    ros::Publisher mjpeg_pub = n.advertise<sensor_msgs::CompressedImage>(
        "/f1g3/image_raw/compressed", 256);

    Mjpeg mjpeg(mjpeg_pub);

    for (int i = 0; ros::ok(); i++) {
        std_msgs::String msg;

        std::stringstream ss;
        ss << "f1g3_dbg#" << std::setw(3) << i << " ";

        rc = f1_loop();
        if (rc < 0) {
            ss << "f1_loop: error";
        } else {
            // ROS_INFO("f1_loop: ok");
            if (rc & (F1L_NEW_IMG_FRAME | F1L_NEW_IR_FRAME))
                ss << "f1_loop:";
            if (rc & F1L_NEW_IMG_FRAME)
                ss << " NEW_IMG_FRAME";
            if (rc & F1L_NEW_IR_FRAME)
                ss << " NEW_IR_FRAME";
            // if (rc & F1L_BUSY)
            //  ROS_INFO("f1_loop: BUSY");
        }

        if (rc & (F1L_NEW_IMG_FRAME | F1L_NEW_IR_FRAME)) {
            msg.data = ss.str();
            ROS_INFO("%s", msg.data.c_str());
            dbg_pub.publish(msg);
        }

        if (rc & F1L_NEW_IMG_FRAME)
            mjpeg.publish();

        /* process callbacks */
        ros::spinOnce();
    }

    return 0;
}
