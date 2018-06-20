/*
这里实现简单的加速度延时开伞
开机后不断读取加速度值，垂直方向加速度大于30M/S2时，启动倒计时
预设的时间到了以后，发出信号驱动MOS管烧断绳索释放弹簧开伞
*/

#include "config.h"
#include "Sensors.h"
#include "LRC328.h"

int Voltage;

uint8_t TimerStart = 0;

uint8_t WakeUp = 0;

uint16_t Timer = 0;

uint8_t RedBlink = 0;

uint8_t rawADC[6];

uint16_t ACC_X, ACC_Y, ACC_Z;

imu_t imu;

unsigned long previous_millis_fast;
unsigned long millis_ONE_HZ, millis_TWO_HZ, millis_FOUR_HZ;

void LED_BLUE_BLINK(){
  if(RedBlink){
    RedBlink = 0;
    digitalWrite(LED_BLUE,LOW);
  }
  else{
    RedBlink = 1;
    digitalWrite(LED_BLUE,HIGH);
  }
}

void LED_BLUE_Indicator(uint8_t num){
  uint8_t i;
  digitalWrite(LED_BLUE,LOW);
  delay(500);
  for(i=0;i<num;i++){
    digitalWrite(LED_BLUE,HIGH);
    delay(180);
    digitalWrite(LED_BLUE,LOW);
    delay(180);
  }
}

void setup(){
  pinMode(KEY, INPUT_PULLUP);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  pinMode(PIN_FIRE, OUTPUT);
  pinMode(VOT_PIN, INPUT);
  analogReference(INTERNAL);
  
  digitalWrite(LED_RED,HIGH);
  digitalWrite(LED_BLUE,HIGH);
  digitalWrite(PIN_FIRE,LOW);
  
  Voltage = analogRead(VOT_PIN);  
   
  #if defined(DEBUG)
    Serial.begin(115200);
    Serial.print("Hello");
    Serial.print("\n"); 
    Serial.print("Voltage Reads: ");
    Serial.print(Voltage); 
    Serial.print("\n"); 
  #endif
  
  initSensors();
  delay(1000);
  
  digitalWrite(LED_RED,LOW);
  digitalWrite(LED_BLUE,LOW);
  
  millis_ONE_HZ = millis();
  millis_TWO_HZ = millis();
  millis_FOUR_HZ = millis();
  previous_millis_fast = millis();
}

void loop()
{
  unsigned long currentMillis;
  currentMillis = millis();  
  //1Hz loop
  if((currentMillis - millis_ONE_HZ) >= 1000)
  {
    millis_ONE_HZ += 1000; 
    if(!WakeUp)
      LED_BLUE_BLINK();
  }  
  //2Hz loop
  if((currentMillis - millis_TWO_HZ) >= 500)
  {
    millis_TWO_HZ += 500; 
    if( WakeUp &&(!TimerStart))
      LED_BLUE_BLINK();
  }
  //4Hz loop
  if((currentMillis - millis_FOUR_HZ) >= 250)
  {
    millis_FOUR_HZ += 250; 
    if(TimerStart)
      LED_BLUE_BLINK();
  }
  //20Hz loop
  if((currentMillis - previous_millis_fast) >= 50)
  {
    previous_millis_fast += 50; 
    
    if((!WakeUp)&&(digitalRead(KEY)==LOW))
    {
      if((imu.accADC[YAW]<400)||(imu.accADC[YAW]>600))
      {
        digitalWrite(LED_RED,HIGH);
        #if defined(DEBUG)
        Serial.print("Acc value reads error");
        Serial.print("\n"); 
        #endif
      }
      else
      {
        digitalWrite(LED_RED,LOW);
        WakeUp = 1;
        #if defined(DEBUG)
        Serial.print("Check ACC value...");
        Serial.print("\n"); 
        #endif
      }
    }
    
    ACC_getADC();
    #if defined(DEBUG)
      Serial.print(imu.accADC[ROLL]); Serial.print("\t"); 
      Serial.print(imu.accADC[PITCH]); Serial.print("\t"); 
      Serial.print(imu.accADC[YAW]); Serial.print("\n"); 
    #endif    
      
    if(WakeUp)
    { 
      if(imu.accADC[YAW] > ACC_Triggle) {
        if(!TimerStart)
          TimerStart = 1;
      }
      if(TimerStart)  Timer++;
  
      if(Timer > TimerDelay)
      {
        #if defined(DEBUG)
        Serial.print("Boom!");
        Serial.print("\n"); 
        #endif
        digitalWrite(LED_BLUE,LOW);
        TimerStart = 0;
        Timer = 0;
        WakeUp = 0;
        digitalWrite(PIN_FIRE,HIGH);
        delay(SwitchOnTime);
        digitalWrite(PIN_FIRE,LOW); 
      }
    }
  }
}
