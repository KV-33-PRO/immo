#include <DynamixelSerial3.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
//#include <sensor_msgs/BatteryState.h>
#include <std_msgs/Bool.h>

//#include <LedControl.h>
//#include <Battery.h>


#define RATE_MS                   50       // задержка для публикации в топик
#define RATE_MS_PARAMS            1000     // задержка для обновления параметров
#define TIME_TO_LIVE_MS           1000     // задержка для остановки движения при отсутствии сообщений в топике cmd_vel
#define RATE_MS_DYNAMIXEL         50       // задержка управления рулем

#define Kp                        167.0    // пропорциональный коэффициент для ПИД регулятора (41.7)
#define Ki                        0.1      // интегральный коэффициент для ПИД регулятора
#define Kd                        0.4      // дифференциальный коэффициент для ПИД регулятора
#define K_SPEED                   0.63     // коэффициент неадекватной скорости

#define WHEEL_DIAMETER            0.151    // диаметр колеса в метрах
#define WHEEL_IMPULSE_COUNT       172.0    // количество импульсов на оборот колеса
#define GRAD_RUDDER               45.0     // угол поворота колес в одну сторону (от центра до максимального выворота (в градусах))
#define DUNAMIXEL_CENTER          517      // значение центрального положения колес (в значениях динамикселя)

#define DYNAMIXEL_ID              19       // идентификатор dynamixel AX-12a
#define RUDDER_TX_PIN             PB10     // выход serial3 TX для подключения Dynamixel AX-12a
#define RUDDER_RX_PIN             PB11     // выход serial3 RX для подключения Dynamixel AX-12a
#define RUDDER_DATA_CONTROL_PIN   PB1     // выход для переключения между TX и RX

#define ENCODER_LEFT_PIN          PB14     // вход с прерыванием для левого энкодера 
#define ENCODER_RIGHT_PIN         PB13     // вход с прерыванием для правого энкодера 
#define DIRECTION_MOTOR_FRONT     PA5      // выход с мотора (для направления движения вперед)
#define DIRECTION_MOTOR_REAR      PA4      // выход с мотора (для направления движения назад)
#define MOTOR_PIN_1               PB8      // выход на драйвер мотора 1
#define MOTOR_PIN_2               PB9      // выход на драйвер мотора 2
#define BUTTON_PIN                PB4      // кнопка старта

#define BAT_BALANCE_PIN_1         PA0      // вход для балансового порта батареи 1
#define BAT_BALANCE_PIN_2         PA1      // вход для балансового порта батареи 2
#define BAT_BALANCE_PIN_3         PA2      // вход для балансового порта батареи 3
#define BAT_BALANCE_PIN_4         PA3      // вход для балансового порта батареи 4

//#define PIN_LEDS    PA7     //Library uses SPI1. Connect the WS2812B data input to MOSI on your board. (STM32: PA7)

#define NUM_JOINTS                3

char *state_names[NUM_JOINTS] = {"left_wheel", "right_wheel", "rudder"};
float state_pos[NUM_JOINTS] = {0, 0, 0};
float state_vel[NUM_JOINTS] = {0, 0, 0};
float state_eff[NUM_JOINTS] = {0, 0, 0};

unsigned long last_ms;
unsigned long last_ms_params;
unsigned long last_msgs_cmd_vel_ms;
unsigned long last_dynamixel_ms;

volatile float enc_left;
volatile float enc_right;
/*
uint8_t battery_pins[4] = {
    BAT_BALANCE_PIN_1,
    BAT_BALANCE_PIN_2,
    BAT_BALANCE_PIN_3,
    BAT_BALANCE_PIN_4
};


Battery battery("/battery", battery_pins, sizeof(battery_pins) / sizeof(uint8_t));
*/
//LedControl led;

ros::NodeHandle nh;

float linear = 0;           //значение для драйвера моторов
float angular = 0;          //значение для руля моторов

float e_prev = 0;           //последнее значение разницы скорости движения
float I_prev = 0;           //последнее значение интегральной составляющей ПИД регулятора

float pid_constants[3];
float cmd_linear;
float cmd_angular;

//Обработка сообщений из топика "cmd_vel"
void drive_cb(const geometry_msgs::Twist& cmd_vel) {
  cmd_angular = cmd_vel.angular.z;
  cmd_linear = -cmd_vel.linear.x;
  last_msgs_cmd_vel_ms = millis();                   //фиксируем время последнего управляющего воздействия
}

sensor_msgs::JointState state_msg;
std_msgs::Bool btn_msg;

ros::Publisher state_pub("joint_states", &state_msg);                    //инициализация издателя топика "joint_states"
ros::Publisher btn_pub("robot_button", &btn_msg);                      //инициализация издателя топика "robot_button"
ros::Subscriber<geometry_msgs::Twist> drive_sub("cmd_vel", drive_cb);    //инициализация подписчика на топик "cmd_vel"

void setup() {
  pinMode(MOTOR_PIN_1, OUTPUT);
  pinMode(MOTOR_PIN_2, OUTPUT);
  pinMode(DIRECTION_MOTOR_FRONT, INPUT);
  pinMode(DIRECTION_MOTOR_REAR, INPUT);
  pinMode(BUTTON_PIN, INPUT);
  
  pinMode(LED_BUILTIN, OUTPUT); 
  attachInterrupt(ENCODER_LEFT_PIN, doEncoderLeft, CHANGE);      //инициализация прерываний для энкодеров
  attachInterrupt(ENCODER_RIGHT_PIN, doEncoderRight, CHANGE);

  //battery.init(nh);
  //led.init(nh);

  Dynamixel.begin(1000000, RUDDER_DATA_CONTROL_PIN);
  /*
  for (int blink = 0; blink < 5; blink++)          //инициализация (мигаем диодом на dynamixel)
  {
    Dynamixel.ledStatus(DYNAMIXEL_ID, ON);
    delay(100);
    Dynamixel.ledStatus(DYNAMIXEL_ID, OFF);
    delay(100);
  }
  */
  angular = DUNAMIXEL_CENTER;
  Dynamixel.move(DYNAMIXEL_ID, angular);       //установить dynamixel в центральное положение

  nh.getHardware()->setBaud(115200);
  nh.initNode();
  
  while(!nh.connected()) {    
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
    digitalWrite(LED_BUILTIN, LOW);
    delay(200);
    nh.spinOnce();
  }
    
  nh.subscribe(drive_sub);
  nh.advertise(state_pub);
  nh.advertise(btn_pub);
  
  state_msg.header.frame_id =  "/driver_states";
  state_msg.name_length = NUM_JOINTS;
  state_msg.velocity_length = NUM_JOINTS;
  state_msg.position_length = NUM_JOINTS;
  state_msg.effort_length = NUM_JOINTS;
  state_msg.name = state_names;
  state_msg.position = state_pos;
  state_msg.velocity = state_vel;
  state_msg.effort = state_eff;
  //Serial.begin(1000000);
}

void update_motors() {
  //Управление движением
  if(millis() - last_msgs_cmd_vel_ms > TIME_TO_LIVE_MS)
  {
    cmd_linear = 0;
  }

  linear = linear2driverMotor(cmd_linear);   //выполняем расчет значения для драйвера двигателей
  driverMotor(linear);

  if(millis() - last_dynamixel_ms > RATE_MS_DYNAMIXEL)
  {
    last_dynamixel_ms = millis();
    angular = angular2dynamixel(cmd_angular);  //выполняем расчет значения для сервомотора
    Dynamixel.move(DYNAMIXEL_ID, angular);     //выполняем поворот динамикселя на нужный угол
    state_pos[2] = dynamixel2angular(Dynamixel.readPosition(DYNAMIXEL_ID)); //Получаем значение положения руля
  }
}

void loop() {
  update_params();
  update_motors();

  unsigned long time_now = millis();
  unsigned long t = time_now - last_ms;
  
  if (t >= RATE_MS) {     //публикуем не чаще чем RATE_MS
    last_ms = millis();   //фиксируем последнее время публикации сообщения в топик
    //Фиксация данных для рассчета одометрии
    state_pos[0] = enc_left;
    state_pos[1] = enc_right;
    
    enc_left = 0;      //обнуляем счетчики
    enc_right = 0;

    state_vel[0] = state_pos[0] * (1000.0 / t); //преобразуем в рад/с
    state_vel[1] = state_pos[1] * (1000.0 / t);

    state_msg.header.stamp = nh.now();     //фиксируем время сообщения

    /*
    //DEBUG INFO
    String str = "T: " + String(millis()) + " L: " + String(state_vel[0]) 
              + " R: " + String(state_vel[1]) 
              + " M: " + String((state_vel[0] + state_vel[1])/2)
              + " C: " + String( cmd_linear )
              + " S: " + String( ((state_vel[0] + state_vel[1])/2) * WHEEL_DIAMETER/2 )
                
              + " ---" 
              + " E: " + String(state_eff[1])
              + " V: " + String(state_eff[2])
              + " ---"
              + " P: "  + String(pid_constants[0])
              + " I: "  + String(pid_constants[1])
              + " D: "  + String(pid_constants[2]);
              
    int str_len = str.length() + 1;
    char char_array[str_len];
    str.toCharArray(char_array, str_len);
    Serial.println(char_array);
    nh.logwarn(char_array);
    */
    state_pub.publish(&state_msg);      //публикуем сообщение в топик
    
    btn_msg.data = digitalRead(BUTTON_PIN);
    btn_pub.publish(&btn_msg);      
    
    //battery.publicBatteryInfo();
    //led.indication();
  }
  nh.spinOnce();
}

inline float impulse2radian(float x) {
  return (x / (WHEEL_IMPULSE_COUNT / 360.0)) * M_PI / 180.0;     //преобразование импульсы > градусы > радианы
}

float angular2dynamixel(float angular) {
  float angular_grad = angular * 180.0 / M_PI; //преобразование радиан > градусы
  float angular_value_res = ((1024.0 / 300.0) * (GRAD_RUDDER)); //значение для динамикселя в одном направлении

  if (angular_grad >= -(GRAD_RUDDER) && angular_grad <= (GRAD_RUDDER))    //если в пределах возможного - ремапим значения для динамикселя
  {
    return map(angular_grad * 100.0, -(GRAD_RUDDER) * 100.0, (GRAD_RUDDER) * 100.0, round(DUNAMIXEL_CENTER - angular_value_res), round(DUNAMIXEL_CENTER + angular_value_res)); //ремапим значения угла поворота на значения динамикселя (с ограничением угла поворота до возможного выворота колес)
  }
  else
  {
    //ограничиваем максимальный поворот динамикселя в обе стороны
    if (angular_grad < -(GRAD_RUDDER))
    {
      return round(DUNAMIXEL_CENTER - angular_value_res);    //программное ограничение поворота колес на максимальный градус влево
    }
    else
    {
      return round(DUNAMIXEL_CENTER + angular_value_res);    //программное ограничение поворота колес на максимальный градус вправо
    }
  }
}

float dynamixel2angular(int dynamixel) {
    float angular_value_res = ((1024.0 / 300.0) * (GRAD_RUDDER)); //значение для динамикселя в одном направлении
    float angular = map(dynamixel, round(DUNAMIXEL_CENTER - angular_value_res), round(DUNAMIXEL_CENTER + angular_value_res), -(GRAD_RUDDER) * 100.0, (GRAD_RUDDER) * 100.0) / 100.0;
    return angular * M_PI / 180.0 ; //преобразование градусы > радиан
}

int linear2driverMotor(float linear_speed)
{
  if(linear_speed == 0){
    I_prev = 0.0;
    e_prev = 0.0;
    return 0;
  }
  
  //Расчет средней скорости движения между публикациями
  float speed_actual = -(state_vel[0]+state_vel[1])/2;
  float e = speed_actual * WHEEL_DIAMETER/2 - linear_speed;       //разница в скорости средней от последней публикации в m/s и желаемая m/s

  //ПИД регулятор для рассчета значения для драйвера моторов
  float P = pid_constants[0] * e;
  float I = I_prev + pid_constants[1] * e;
  float D = pid_constants[2] * (e - e_prev);
  float motor_value = round(P + I + D);
  
  if((motor_value < 0 && linear_speed < 0) || (motor_value > 0 && linear_speed > 0))
    motor_value = 0;
  

  //Для отладки
  state_eff[1] = e;
  state_eff[2] = motor_value;
  
  I_prev = I;                     //фиксируем интегральную составляющую
  e_prev = e;                     //фиксируем последнее значение разницы в скорости
  
  int motor_val_min = 45;
  
  if(motor_value < 0 && motor_value >= -motor_val_min){
    motor_value = -motor_val_min;
  } 
  if(motor_value > 0 && motor_value <= motor_val_min){
    motor_value = motor_val_min;
  }
  
  //Убираем переполнение ШИМ
  if (motor_value>255){
    return 255;  
  }
  
  if (motor_value<-255){
    return -255;  
  }
  
  return motor_value;
}

//Обработчик управления моторами
void driverMotor(float linear){
    if (linear > 1) {
      analogWrite(MOTOR_PIN_1, linear);        //запускаем моторы направление движения - вперед
      analogWrite(MOTOR_PIN_2, 0);
    }
    else if (linear < -1) {
      analogWrite(MOTOR_PIN_2, -linear);       //запускаем моторы направление движения - назад
      analogWrite(MOTOR_PIN_1, 0);
    }
    else if (linear == 0) {
      analogWrite(MOTOR_PIN_2, 0);             //отключаем моторы
      analogWrite(MOTOR_PIN_1, 0);
    }
}

// Обработчики прерываний для левого и правого энкодеров
void doEncoderLeft() {
  enc_left += impulse2radian(getEncoderCount());
}

void doEncoderRight() {
  enc_right += impulse2radian(getEncoderCount());
}

// Обработчик направления движения
float getEncoderCount()
{
  if (linear != 0) {    //если сигнал движения есть, то ориентируемся в направлении с помощью него
    if (linear > 0) {
      return 1.0;
    }
    if (linear < 0) {
      return -1.0;
    }
  } else {              //нет сигнала движения, но движемся (по инерции или толкают)... ориентируемся по датчику...
    if (digitalRead(DIRECTION_MOTOR_FRONT) == HIGH && digitalRead(DIRECTION_MOTOR_REAR) == LOW)
    {
      return 1.0;
    }
    if (digitalRead(DIRECTION_MOTOR_FRONT) == LOW && digitalRead(DIRECTION_MOTOR_REAR) == HIGH)
    {
      return -1.0;      //движение назад
    }
  }
}

void update_params() {
  unsigned long tp = millis() - last_ms_params;
  if (tp >= RATE_MS_PARAMS) {     
    if (!nh.getParam("/pid", pid_constants, 3)){ 
       //default values
       pid_constants[0]= Kp;
       pid_constants[1]= Ki;
       pid_constants[2]= Kd; 
    } 
    last_ms_params = millis(); 
  }
}
