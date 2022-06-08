// Own
#include "flirone.h"

// ROS
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>

// C++
#include <sstream>
#include <vector>

// C
#include <sys/time.h>

class ImageMsg
{
public:
    virtual void publish(struct f1_frame *frame, const struct timeval &tv) = 0;

protected:
    void fill_header(std_msgs::Header &hdr, const std::string &id,
        const struct timeval &tv)
    {
        hdr.seq = m_seq++;
        hdr.stamp.sec = tv.tv_sec;
        hdr.stamp.nsec = tv.tv_usec * 1000;
        hdr.frame_id = id;
    }

private:
    uint32_t m_seq = 1;
};

// MJPEG publisher
class Mjpeg : public ImageMsg
{
public:
    Mjpeg(ros::Publisher &pub) : m_pub(pub)
    {}

    void publish(struct f1_frame *frame, const struct timeval &tv)
    {
        sensor_msgs::CompressedImage msg;

        fill_header(msg.header, "optical_camera", tv);
        msg.format = "jpeg";
        msg.data = std::vector<unsigned char>(frame->jpg_ptr,
            frame->jpg_ptr + frame->jpg_sz);
        m_pub.publish(msg);
    }

private:
    ros::Publisher &m_pub;
};

// Thermal images publisher
class Thermal : public ImageMsg
{
public:
    Thermal(ros::Publisher &pub) : m_pub(pub)
    {}

    void publish(struct f1_frame *frame, const struct timeval &tv)
    {
        sensor_msgs::Image msg;

        fill_header(msg.header, "thermal_camera", tv);
        msg.height = frame->ir_height;
        msg.width = frame->ir_width;
        msg.encoding = "rgb8";
        msg.is_bigendian = false;
        msg.step = msg.width * 3;
        msg.data = std::vector<unsigned char>(frame->ir_ptr,
            frame->ir_ptr + frame->ir_sz);
        m_pub.publish(msg);
    }

private:
    ros::Publisher &m_pub;
};

int main(int argc, char **argv)
{
    struct f1_cfg f1cfg = {};
    struct f1_frame *f1_frame;
    int rc;

    // Init flirone
    f1cfg.pal_path = "palettes/Rainbow.raw";
    f1_frame = f1_init(&f1cfg);
    if (f1_frame == NULL)
        return -1;

    // Init ROS
    ros::init(argc, argv, "flir_one_g3_node");
    ros::NodeHandle n;

    // Setup publishers
    // (topic, msg_queue_size)
    ros::Publisher dbg_pub = n.advertise<std_msgs::String>("/f1g3/dbg", 1000);
    ros::Publisher mjpeg_pub = n.advertise<sensor_msgs::CompressedImage>(
        "/f1g3/optical/image_raw/compressed", 256);
    ros::Publisher thermal_pub = n.advertise<sensor_msgs::Image>(
        "/f1g3/thermal/image_raw", 256);

    Mjpeg mjpeg(mjpeg_pub);
    Thermal thermal(thermal_pub);

    // main loop
    for (int i = 0; ros::ok(); i++) {
        struct timeval tv = {};
        std_msgs::String msg;

        std::stringstream ss;
        ss << "f1g3_dbg#" << std::setw(3) << i << " ";

        // run flirone and check results
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
            gettimeofday(&tv, NULL);

            msg.data = ss.str();
            ROS_INFO("%s", msg.data.c_str());
            dbg_pub.publish(msg);
        }

        // publish images
        if (rc & F1L_NEW_IMG_FRAME)
            mjpeg.publish(f1_frame, tv);
        if (rc & F1L_NEW_IR_FRAME)
            thermal.publish(f1_frame, tv);

        /* process callbacks */
        ros::spinOnce();
    }

    return 0;
}
