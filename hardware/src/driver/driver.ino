#include <DynamixelSerial3.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>

#define RATE_MS                   20       // задержка для публикации в топик

#define Kp                        60.0     // пропорциональный коэффициент для ПИД регулятора
#define Ki                        0.0      // интегральный коэффициент для ПИД регулятора
#define Kd                        0.0      // дифференциальный коэффициент для ПИД регулятора   

#define WHEEL_DIAMETER            0.151    // диаметр колеса в метрах
#define WHEEL_IMPULSE_COUNT       172.0    // количество импульсов на оборот колеса
#define GRAD_RUDDER               45.0     // угол поворота колес в одну сторону (от центра до максимального выворота (в градусах))

#define DYNAMIXEL_ID              3        // идентификатор dynamixel AX-12a
#define RUDDER_TX_PIN             PB10     // выход serial3 TX для подключения Dynamixel AX-12a
#define RUDDER_RX_PIN             PB11     // выход serial3 RX для подключения Dynamixel AX-12a
#define RUDDER_DATA_CONTROL_PIN   PB12     // выход для переключения между TX и RX (для дуплексной связи)

#define ENCODER_LEFT_PIN          PA6      // вход с прерыванием для левого энкодера 
#define ENCODER_RIGHT_PIN         PA7      // вход с прерыванием для правого энкодера 
#define DIRECTION_MOTOR_FRONT     PA5      // выход с мотора (для направления движения вперед)
#define DIRECTION_MOTOR_REAR      PA4      // выход с мотора (для направления движения назад)
#define MOTOR_PIN_1               PB8      // выход на драйвер мотора 1
#define MOTOR_PIN_2               PB9      // выход на драйвер мотора 2

#define NUM_JOINTS                3

char *state_names[NUM_JOINTS] = {"left_wheel", "right_wheel", "rudder"};
float state_pos[NUM_JOINTS] = {0, 0, 0};
float state_vel[NUM_JOINTS] = {0, 0, 0};
float state_eff[NUM_JOINTS] = {0, 0, 0};
unsigned long last_ms;

ros::NodeHandle nh;

float linear = 0;           //значение для драйвера моторов
float e_prev = 0;           //последнее значение разницы скорости движения
float I_prev = 0;           //последнее значение интегральной составляющей ПИД регулятора
float speed_actual = 0;     //текущая усредненая скорость (между публикациями)

//Обработка сообщений из топика "cmd_vel"
void drive_cb(const geometry_msgs::Twist& cmd_vel) {
  Dynamixel.move(DYNAMIXEL_ID, angular2dynamixel(cmd_vel.angular.z));      //выполняем поворот динамикселя на нужный угол
  
  // ЗАГЛУШКА
  //linear = map(cmd_vel.linear.x * 1000, -1000, 1000, -255, 255);         //ремапим значения из топика в значения для моторов
  
  linear = linear2driverMotor(cmd_vel.linear.x);                           //выполняем расчет значения для драйвера двигателей
  
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

sensor_msgs::JointState state_msg;

ros::Publisher state_pub("joint_states", &state_msg);                    //инициализация издателя топика "joint_states"
ros::Subscriber<geometry_msgs::Twist> drive_sub("cmd_vel", drive_cb);    //инициализация подписчика на топик "cmd_vel"

void setup() {
  pinMode(MOTOR_PIN_1, OUTPUT);
  pinMode(MOTOR_PIN_2, OUTPUT);
  pinMode(DIRECTION_MOTOR_FRONT, INPUT);
  pinMode(DIRECTION_MOTOR_REAR, INPUT);
  attachInterrupt(ENCODER_LEFT_PIN, doEncoderLeft, CHANGE);      //инициализация прерываний для энкодеров
  attachInterrupt(ENCODER_RIGHT_PIN, doEncoderRight, CHANGE);

  Dynamixel.begin(1000000, RUDDER_DATA_CONTROL_PIN);
  for (int blink = 0; blink < 5; blink++)          //инициализация (мигаем диодом на dynamixel)
  {
    Dynamixel.ledStatus(DYNAMIXEL_ID, ON);
    delay(100);
    Dynamixel.ledStatus(DYNAMIXEL_ID, OFF);
    delay(100);
  }
  Dynamixel.move(DYNAMIXEL_ID, 512);       //установить dynamixel в центральное положение

  nh.getHardware()->setBaud(500000);
  nh.initNode();
  nh.subscribe(drive_sub);
  nh.advertise(state_pub);

  state_msg.header.frame_id =  "/driver_states";
  state_msg.name_length = NUM_JOINTS;
  state_msg.velocity_length = NUM_JOINTS;
  state_msg.position_length = NUM_JOINTS;
  state_msg.effort_length = NUM_JOINTS;
  state_msg.name = state_names;
  state_msg.position = state_pos;
  state_msg.velocity = state_vel;
  state_msg.effort = state_eff;
}

void loop() {
  unsigned long t = millis() - last_ms;
  if (t >= RATE_MS) {     //публикуем не чаще чем RATE_MS
    last_ms = millis();   //фиксируем последнее время публикации сообщения в топик
    state_vel[0] = state_pos[0] / (t / 1000.0); //преобразуем в рад/с
    state_vel[1] = state_pos[1] / (t / 1000.0);
    state_msg.header.stamp = nh.now();     //фиксируем время сообщения
    state_pub.publish(&state_msg);      //публикуем сообщение в топик
    state_pos[0] = 0;      //обнуляем счетчики
    state_pos[1] = 0;

    //Расчет средней скорости движения между публикациями
    if(speed_actual==0.0){
    speed_actual += (state_vel[0]+state_vel[1])/2;
    }
    else
    {
    speed_actual = (speed_actual+((state_vel[0]+state_vel[1])/2))/2;
    }
  }
  nh.spinOnce();
}

float impulse2radian(float x) {
  return (x / (WHEEL_IMPULSE_COUNT / 360.0)) * M_PI / 180.0;     //преобразование импульсы > градусы > радианы
}

float angular2dynamixel(float angular) {
  float angular_grad = angular * 180.0 / M_PI; //преобразование радиан > градусы
  float angular_value_res = ((1024.0 / 300.0) * (GRAD_RUDDER)); //значение для динамикселя в одном направлении

  if (angular_grad >= -(GRAD_RUDDER) && angular_grad <= (GRAD_RUDDER))    //если в пределах возможного - ремапим значения для динамикселя
  {
    state_pos[2] = angular;     //фиксируем положение динамикселя для сообщения
    return map(angular_grad * 100.0, -(GRAD_RUDDER) * 100.0, (GRAD_RUDDER) * 100.0, round(512 - angular_value_res), round(512 + angular_value_res)); //ремапим значения угла поворота на значения динамикселя (с ограничением угла поворота до возможного выворота колес)
  }
  else
  {
    //ограничиваем максимальный поворот динамикселя в обе стороны
    if (angular_grad < -(GRAD_RUDDER))
    {
      state_pos[2] = -GRAD_RUDDER * M_PI / 180.0;    //фиксируем положение динамикселя для сообщения
      return round(512 - angular_value_res);    //программное ограничение поворота колес на максимальный градус влево
    }
    else
    {
      state_pos[2] = GRAD_RUDDER * M_PI / 180.0;     //фиксируем положение динамикселя для сообщения
      return round(512 + angular_value_res);    //программное ограничение поворота колес на максимальный градус вправо
    }
  }
}

int linear2driverMotor(float linear_speed)
{
  if(linear_speed == 0){
    I_prev = 0.0;
    e_prev = 0.0;
    speed_actual = 0.0;
    return 0;
  }
  
// TODO
//  if (speed_actual==0.0){
//    return linear*255;
//  }
  
  float e = (speed_actual*WHEEL_DIAMETER*M_PI) - linear_speed;       //разница в скорости средней от последней публикации в m/s и желаемая m/s

  //ПИД регулятор для рассчета значения для драйвера моторов
  float P = Kp * e;
  float I = I_prev + Ki * e;
  float D = Kd * (e - e_prev);
  float motor_value = round(P + I + D);
  
  I_prev = I;                     //фиксируем интегральную составляющую
  e_prev = e;                     //фиксируем последнее значение разницы в скорости
  speed_actual = 0.0;             //обнуляем значение средней скорости за период обновления cmd_vel в рад в сек...

  //Для отладки
  state_eff[2]=motor_value;                    

  //Убираем переполнение ШИМ
  if (motor_value>255){
    return 255;  
  }
  if (motor_value<-255){
    return -255;  
  }
  
  return motor_value;
}

// Обработчики прерываний для левого и правого энкодеров
void doEncoderLeft() {
  state_pos[0] += impulse2radian(getEncoderCount());
}
void doEncoderRight() {
  state_pos[1] += impulse2radian(getEncoderCount());
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
