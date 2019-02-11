#ifndef Led_control_h
#define Led_control_h
#include <ros.h>


class LedControl {
public:
    LedControl();
    void init(ros::NodeHandle &nh);
    void led();

private:
    void head_cb(const std_msgs::UInt8& mode);
    void turn_cb(const std_msgs::UInt8& mode);
    void back_cb(const std_msgs::UInt8& mode);
    void brake_cb(const std_msgs::UInt8& mode);
    void flasher_cb(const std_msgs::UInt8& mode);
    void position_cb(const std_msgs::UInt8& mode);

    ros::Subscriber<std_msgs::UInt8> _head_sub;
    ros::Subscriber<std_msgs::UInt8> _back_sub;
    ros::Subscriber<std_msgs::UInt8> _turn_sub;
    ros::Subscriber<std_msgs::UInt8> _brake_sub;
    ros::Subscriber<std_msgs::UInt8> _flasher_sub;
    ros::Subscriber<std_msgs::UInt8> _position_sub;
};

#endif
