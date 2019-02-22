#ifndef LedControl_h
#define LedControl_h
#include <ros.h>
#include <std_msgs/UInt8.h>
#include <LedWS2812BforSTM32.h>

#define COUNT_TOPICS              6
#define COUNT_INDICATION_STATUS   8

class LedControl {
private:
    void position_cb(const std_msgs::UInt8& mode);
    void head_cb(const std_msgs::UInt8& mode);
    void turn_cb(const std_msgs::UInt8& mode);
    void back_cb(const std_msgs::UInt8& mode);
    void brake_cb(const std_msgs::UInt8& mode);
    void flasher_cb(const std_msgs::UInt8& mode);
    bool statusChanged(int strIndication);
    ros::Subscriber<std_msgs::UInt8, LedControl> _head_sub;
    ros::Subscriber<std_msgs::UInt8, LedControl> _back_sub;
    ros::Subscriber<std_msgs::UInt8, LedControl> _turn_sub;
    ros::Subscriber<std_msgs::UInt8, LedControl> _brake_sub;
    ros::Subscriber<std_msgs::UInt8, LedControl> _flasher_sub;
    ros::Subscriber<std_msgs::UInt8, LedControl> _position_sub;
    ros::NodeHandle *_nh;
    std_msgs::UInt8 _indication[COUNT_TOPICS][2];
    bool _indication_status[COUNT_INDICATION_STATUS];
public:
    LedControl();
    void init(ros::NodeHandle &nh);
    void indication();
};

#endif
