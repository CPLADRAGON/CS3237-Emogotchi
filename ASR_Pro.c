#include "asr.h"
extern "C"{ void * __dso_handle = 0 ;}
#include "setup.h"
#include "myLib/asr_event.h"

uint32_t snid;
uint8_t var = 0;
uint32_t playid = 0;
uint32_t ser_rec = 0;
uint32_t temp = 0;
uint32_t id2 = 0;
QueueHandle_t play=NULL;
void ASR_CODE();
void play_app();

//{speak:Mia,vol:4,speed:4,platform:haohaodada,version:V3}
//{playid:10001,voice:Hello I am Emogochi Your smart emotion awear friend I will help you under stand yourself better}
//{playid:10002,voice:I am resting now Call me anytime you need me}

//{ID:10250,keyword:"B",ASR:"MAX-VOICE",ASRTO:"Adjust the volume to maximum"}
//{ID:10251,keyword:"B",ASR:"MIDDLE-VOICE",ASRTO:"Adjust the volume to medium"}
//{ID:10252,keyword:"B",ASR:"MIN-VOICE",ASRTO:"Adjust the volume to the minimum"}

void ASR_CODE(){
  set_state_enter_wakeup(60000);
  switch (snid) {
   case 1:
    digitalWrite(4,1);
    break;
   case 2:
    digitalWrite(4,0);
    break;
   case 3:
    digitalWrite(5,1);
    break;
   case 4:
    digitalWrite(5,0);
    break;
   case 5:
    digitalWrite(3,1);
    break;
   case 6:
    digitalWrite(3,0);
    break;
   case 7:
    //{playid:10500,voice:Music1}
    play_audio(10500);
    break;
   case 8:
    digitalWrite(3,0);
    digitalWrite(4,0);
    digitalWrite(5,0);
    break;
   case 11:
    break;
  }

}

void GPIO0_irq(){
  if(gpio_get_irq_status(1)){
    Clear_GPIO_irq(1);
    temp = millis();
    while (millis() - temp < 500) {
    }
    ser_rec = 24;
    var = xQueueSendFromISR(play,&ser_rec,1000000);
  }

}

void play_app(){
  while (1) {
    if(xQueueReceive(play,&playid,0)){
      delay(200);
      enter_wakeup(15000);
      delay(200);
      //{playid:10501,voice:I sense your stress level is rising. Let’s take a short break, okay}
      play_audio(10501);
      //{playid:10502,voice:I will dim the light and play some soft music for you}
      play_audio(10502);
      digitalWrite(2,1);
      //{playid:10503,voice:Music2}
      play_audio(10503);
      delay(5000);
      digitalWrite(2,0);
      //{playid:10504,voice:Glad you are okay! Just dont overwork, alright}
      play_audio(10504);
      delay(5000);
    }
    delay(10);
  }
  vTaskDelete(NULL);
}

void hardware_init(){
  play=xQueueCreate(10,4);
  vol_set(3);
  xTaskCreate(play_app,"play_app",256,NULL,4,NULL);
  vTaskDelete(NULL);
}

void setup()
{
  //{ID:0,keyword:"A",ASR:"Hello emo",ASRTO:"I'm here"}
  //{ID:1,keyword:"B",ASR:"Open window",ASRTO:"Window is open"}
  //{ID:2,keyword:"B",ASR:"Close window",ASRTO:"Window is closed"}
  //{ID:3,keyword:"B",ASR:"Light on",ASRTO:"Light is on"}
  //{ID:4,keyword:"B",ASR:"Light off",ASRTO:"Light is off"}
  //{ID:5,keyword:"B",ASR:"Open the door",ASRTO:"Door is open"}
  //{ID:6,keyword:"B",ASR:"Close the door",ASRTO:"Door is closed"}
  //{ID:7,keyword:"B",ASR:"Play music",ASRTO:"Playing music"}
  //{ID:8,keyword:"B",ASR:"Good night",ASRTO:"Have a good sleep."}
  //{ID:9,keyword:"B",ASR:"I am tired",ASRTO:"Alright, lets slow down for a bit.."}
  //{ID:10,keyword:"B",ASR:"I am sad",ASRTO:"Lets take a short break, okay"}
  //{ID:11,keyword:"B",ASR:"Cheer me up",ASRTO:"I am here with you. Lets listen to something cheerful together."}
  setPinFun(1,FIRST_FUNCTION);
  pinMode(1,input);
  dpmu_set_adio_reuse(PA1,DIGITAL_MODE);
  setPinFun(2,FIRST_FUNCTION);
  pinMode(2,output);
  setPinFun(3,FIRST_FUNCTION);
  pinMode(3,output);
  setPinFun(4,FIRST_FUNCTION);
  pinMode(4,output);
  setPinFun(5,FIRST_FUNCTION);
  pinMode(5,output);
  pinMode(1,input);
  Set_GPIO_irq(1,up_edges_trigger,GPIO0_irq);
}
