#include <ros.h>
#include <LedWS2812BforSTM32.h>

#define RATE_MS                   20       // задержка для публикации в топик
#define RATE_CHANGE_MS            3000     // задержка для смена режима работы

unsigned long last_ms;
unsigned long last_change_ms;
int change_count = -1;
bool twin = false;
bool twin_turn = false;

ros::NodeHandle nh;
bool indication_status[8];

void setup() {
  nh.initNode();
  Led.init();
  Serial.begin(115200);
}

void loop() {
    changeMode();

    if (millis() - last_ms >= RATE_MS) {     //публикуем не чаще чем RATE_MS
        last_ms = millis();                    //фиксируем последнее время публикации сообщения в топик
        Led.indication(indication_status);

        Serial.println(Led.getData());    //Для вывода индикации в консоль. Для отладки
    }
    nh.spinOnce();
}

void changeMode(){
    if (millis() - last_change_ms >= RATE_CHANGE_MS)
    {
        last_change_ms = millis();
        if(change_count!=-1){
            if(change_count < 8){
                indication_status[change_count]=true;
                if (change_count == 4)
                {
                    if (twin == false)
                    {
                    indication_status[3]=false;
                    twin = true;
                    change_count--;
                    }
                    else
                    {
                    indication_status[3]=true;
                    }
                }
                if (change_count == 6)
                {
                    if (twin_turn == false)
                    {
                    indication_status[5]=false;
                    twin_turn = true;
                    change_count--;
                    }
                    else
                    {
                    indication_status[5]=true;
                    }
                }
                change_count++;
            }
            else
            {
                change_count = -1;
                twin=false;
                twin_turn=false;
            }
        }
        else
        {
            for(int i = 0; i<8; i++)
            {
                indication_status[i]=false;
            }
            change_count++;
        }
    }
}
