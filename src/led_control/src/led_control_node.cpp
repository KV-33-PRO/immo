#include "ros/ros.h"
#include <std_msgs/UInt8.h>
#include <geometry_msgs/Twist.h>

//TURN
#define TURN_OFF   0
#define TURN_LEFT  1
#define TURN_RIGHT 2
#define TURN_FLASH 3

//HEAD
#define HEAD_OFF         0
#define HEAD_DIPPED_BEAM 1
#define HEAD_HIGH_BEAM   2

//BACK
#define BACK_OFF 0
#define BACK_ON  1

//BRAKE
#define BRAKE_OFF 0
#define BRAKE_ON  1

//FLASHER
#define FLASHER_OFF 0
#define FLASHER_ON  1

//POSITION_LAMPS
#define POSITION_LAMPS_OFF 0
#define POSITION_LAMPS_ON  1


#define TIME_PARKING      10    //Время задержки до индикации аварийки
#define ANGULAR_LIMIT     0.1   //Предел для включения поворотников
#define LINEAR_LIMIT      0.05  //Предел для включения стопов

class LedControl{
private:
    ros::Subscriber cmd_vel_sub;
    ros::Publisher head_pub;
    ros::Publisher back_pub;
    ros::Publisher turn_pub;
    ros::Publisher brake_pub;
    ros::Publisher flasher_pub;
    ros::Publisher position_pub;
    ros::NodeHandle nh;

    std_msgs::UInt8 head;
    std_msgs::UInt8 back;
    std_msgs::UInt8 turn;
    std_msgs::UInt8 brake;
    std_msgs::UInt8 flasher;
    std_msgs::UInt8 position;

    ros::Time parking_time;
    bool parking;
    bool not_moving;

    double linear_prev;
    double linear;
    double angular;

    uint8_t turnMode()
    {
        uint8_t mode = TURN_OFF;
        //Определение состояния парковки
        if(linear==0.0)
        {
            if(not_moving==true)
            {
                if (parking_time.toSec()+TIME_PARKING < ros::Time::now().toSec())
                {
                    //Включаем аварийный сигнал
                    mode = TURN_FLASH;
                    parking=true;
                }
            }
            else
            {
                //Фиксируем режим и время остановки
                parking_time=ros::Time::now();
                not_moving=true;
            }
        }
        else
        {
            not_moving=false;
            parking=false;
            parking_time=ros::Time::now();

            if(angular > ANGULAR_LIMIT)
            {
                //Включаем поворотник вправо
                mode = TURN_RIGHT;
            }
            if(angular < -ANGULAR_LIMIT)
            {
                //Включаем поворотник влево
                mode = TURN_LEFT;
            }
        }
        return mode;
    }

    uint8_t brakeMode()
    {
        uint8_t mode = BRAKE_OFF;
        if(linear > 0 && linear < (linear_prev - LINEAR_LIMIT) ||
           linear < 0 && linear > (linear_prev + LINEAR_LIMIT))
        {
            //Включаем стоп сигнал
            mode = BRAKE_ON;
        }
        linear_prev = linear;
        return mode;
    }

    uint8_t backMode()
    {
        //Включаем задний ход при движении назад
        if(linear < 0)
        {
            return BACK_ON;
        }
        else
        {
            return BACK_OFF;
        }
    }

    uint8_t headMode(uint8_t head_mode)
    {
        uint8_t mode = HEAD_OFF;

        //При движении фары включены, в режиме парковки головной свет отключен
        if(parking!=true)
        {
            if(position.data==POSITION_LAMPS_ON)
            {
                //Включаем ближний свет
                if(head_mode == HEAD_DIPPED_BEAM)
                {
                    mode = HEAD_DIPPED_BEAM;
                }
                //Включаем дальний свет
                if(head_mode == HEAD_HIGH_BEAM)
                {
                    mode = HEAD_HIGH_BEAM;
                }
            }
        }
        return mode;
    }

    uint8_t positionMode(uint8_t position_mode)
    {
        if(position_mode==POSITION_LAMPS_ON)
        {
            //Включаем габариты
            return POSITION_LAMPS_ON;
        }
        else
        {
            //Выключаем габариты
            return POSITION_LAMPS_OFF;
        }
    }

    uint8_t flesherMode(uint8_t flasher_mode)
    {
        if(flasher_mode==FLASHER_ON)
        {
            //Включаем мигалку
            return FLASHER_ON;
        }
        else
        {
            //Выключаем мигалку
            return FLASHER_OFF;
        }
    }

public:
    LedControl(ros::NodeHandle &n){
        nh = n;
        cmd_vel_sub = nh.subscribe("cmd_vel", 10, &LedControl::cmd_velCallBack, this);
        head_pub = nh.advertise<std_msgs::UInt8>("light_control/head", 10);
        back_pub = nh.advertise<std_msgs::UInt8>("light_control/back", 10);
        turn_pub = nh.advertise<std_msgs::UInt8>("light_control/turn", 10);
        brake_pub = nh.advertise<std_msgs::UInt8>("light_control/brake", 10);
        flasher_pub = nh.advertise<std_msgs::UInt8>("light_control/flasher", 10);
        position_pub = nh.advertise<std_msgs::UInt8>("light_control/position", 10);
        parking = false;
        not_moving=false;
    }

    void publicInfo(){
        ros::spinOnce();
        ros::Rate r(10);

        //Режим работы габаритных огней
        position.data = positionMode(POSITION_LAMPS_ON);

        //Режим работы головного света
        head.data = headMode(HEAD_HIGH_BEAM);

        //Режим работы поворотников
        turn.data = turnMode();

        //Режим работы стопов
        brake.data = brakeMode();

        //Режим работы заднего хода
        back.data = backMode();

        //Режим работы мигалки
        flasher.data = flesherMode(FLASHER_OFF);

        //Публикация значений в топики
        head_pub.publish(head);
        back_pub.publish(back);
        turn_pub.publish(turn);
        brake_pub.publish(brake);
        flasher_pub.publish(flasher);
        position_pub.publish(position);

        //ROS_INFO("head: %i, back: %i, turn: %i, brake: %i, flasher: %i, position_lamps: %i", head.data, back.data, turn.data, brake.data, flasher.data, position_lamps.data);

        r.sleep();
    }

    void cmd_velCallBack(const geometry_msgs::Twist &cmd_vel_msg){
        angular = cmd_vel_msg.angular.z;
        linear = cmd_vel_msg.linear.x;
    }

};

int main(int argc, char **argv) {
    ros::init(argc, argv, "led_control_node");
    ros::NodeHandle n;
    LedControl led(n);

    while(ros::ok()){
        led.publicInfo();
    }
    return(0);
}
