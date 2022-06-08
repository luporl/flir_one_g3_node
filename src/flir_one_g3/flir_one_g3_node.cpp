// Own
#include "flirone.h"

// ROS
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>

// C++
#include <iomanip>
#include <sstream>
#include <vector>

// C
#include <libgen.h>
#include <sys/time.h>

#define NODE_NAME   "flir_one_g3"
#define PALETTE     "Rainbow.raw"


/* helpers to allow flirone to use ROS */
extern "C" {

static char strbuf[1024];

void ros_info(const char *fmt, ...)
{
    va_list ap;

    va_start(ap, fmt);
    vsprintf(strbuf, fmt, ap);
    va_end(ap);
    ROS_INFO("%s", strbuf);
}

void ros_error(const char *fmt, ...)
{
    va_list ap;

    va_start(ap, fmt);
    vsprintf(strbuf, fmt, ap);
    va_end(ap);
    ROS_ERROR("%s", strbuf);
}

}

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

// Thermal info publisher
class ThermalInfo : public ImageMsg
{
public:
    ThermalInfo(ros::Publisher &pub) : m_pub(pub)
    {}

    void publish(struct f1_frame *frame, const struct timeval &tv)
    {
        std_msgs::String msg;
        std::ostringstream ss;

        ss << "min/med/max temps: ";

        ss << std::fixed << std::setprecision(1) << std::setfill('0')
           << frame->temp_min << "/"
           << frame->temp_med << "/"
           << frame->temp_max;

        ss << " max @ " << frame->temp_max_x << ", " << frame->temp_max_y;

        msg.data = ss.str();
        m_pub.publish(msg);
        ROS_DEBUG("%s", msg.data.c_str());
    }

private:
    ros::Publisher &m_pub;
};

int main(int argc, char **argv)
{
    struct f1_cfg f1cfg = {};
    struct f1_frame *f1_frame;
    char *pal;
    int rc;

    // find palettes path
    strncpy(strbuf, argv[0], sizeof(strbuf));
    strbuf[sizeof(strbuf) - 1] = 0;
    pal = strbuf;
    if (strlen(pal) > 512) {
        ROS_ERROR("Executable path is too long");
        return -1;
    }
    // go back 3 levels to get to install dir
    for (int i = 0; i < 3; i++) {
        pal = dirname(pal);
        if (strcmp(pal, ".") == 0 || strcmp(pal, "/") == 0) {
            ROS_ERROR("Couldn't find path to palettes dir");
            return -1;
        }
    }
    // then append relative palette path
    strcat(pal, "/share/flir_one_g3/palettes/" PALETTE);

    // Init ROS
    ros::init(argc, argv, "flir_one_g3_node");
    ros::NodeHandle n;

    // Init flirone
    ROS_INFO("Palette path: %s", pal);
    f1cfg.pal_path = pal;
    // f1cfg.pal_colors = 1;
    // f1cfg.pal_inv = 1;
    f1_frame = f1_init(&f1cfg);
    if (f1_frame == NULL)
        return -1;

    // Setup publishers
    // (topic, msg_queue_size)
    ros::Publisher dbg_pub = n.advertise<std_msgs::String>(
        "/" NODE_NAME "/dbg", 1000);
    ros::Publisher mjpeg_pub = n.advertise<sensor_msgs::CompressedImage>(
        "/" NODE_NAME "/optical/image_raw/compressed", 256);
    ros::Publisher thermal_pub = n.advertise<sensor_msgs::Image>(
        "/" NODE_NAME "/thermal/image_raw", 256);
    ros::Publisher thermal_info_pub = n.advertise<std_msgs::String>(
        "/" NODE_NAME "/thermal/info", 1000);

    Mjpeg mjpeg(mjpeg_pub);
    Thermal thermal(thermal_pub);
    ThermalInfo thermal_info(thermal_info_pub);

    // main loop
    for (int i = 0; ros::ok(); i++) {
        struct timeval tv = {};
        std_msgs::String msg;
        std::stringstream ss;

        ss << "msg#" << std::setw(3) << i << " ";

        // run flirone and check results
        rc = f1_loop();
        if (rc < 0) {
            ss << "ERROR acquiring frame";
        } else {
            if (rc & F1L_NEW_IMG_FRAME)
                ss << "NEW_IMG_FRAME";
            if (rc & F1L_NEW_IR_FRAME)
                ss << " NEW_IR_FRAME";
            // if (rc & F1L_BUSY)
            //  ss << " BUSY";
        }

        if (rc < 0 || (rc & (F1L_NEW_IMG_FRAME | F1L_NEW_IR_FRAME))) {
            gettimeofday(&tv, NULL);

            msg.data = ss.str();
            ROS_DEBUG("%s", msg.data.c_str());
            dbg_pub.publish(msg);
        }

        // publish images
        if (rc & F1L_NEW_IMG_FRAME)
            mjpeg.publish(f1_frame, tv);
        if (rc & F1L_NEW_IR_FRAME) {
            thermal.publish(f1_frame, tv);
            thermal_info.publish(f1_frame, tv);
        }

        /* process callbacks */
        ros::spinOnce();
    }

    return 0;
}
