/*


*/

#include <Arduino.h>
#include <TireWarmer.h>
#include<ADS1115_WE.h> 
#include<Wire.h>
#define I2C_ADDRESS 0x48

ADS1115_WE adc = ADS1115_WE(I2C_ADDRESS);

float readChannel(ADS1115_MUX channel) {
  float voltage = 0.0;
  adc.setCompareChannels(channel);
  voltage = adc.getResult_V(); // alternative: getResult_mV for Millivolt
  return voltage;
}

WarmerDisp warmer(0x20, 1);

const uint16_t framerate = 5;//LCDのフレームレート Hz
uint16_t lastsecond = 0;
uint16_t lastframe = 0;
uint16_t sps = 0;

float P, I, D;
float dP, dI, dD, predP;
const float Kp = 8;    //5.5
const float Ki = 0.2;  //0.3
const float Kd = 0.08; //0.1

const byte HEATER_FR_PIN = PIN_A1; //Heater control output
const byte HEATER_FL_PIN = PIN_A2;
const byte HEATER_RR_PIN = PIN_A3;
const byte HEATER_RL_PIN = PIN_A6;
const byte SOUND_PIN = PIN_A0;



void setup()
{
  Serial.begin(115200);
  warmer.init(HEATER_FR_PIN, HEATER_FL_PIN, HEATER_RR_PIN, HEATER_RL_PIN, SOUND_PIN);

  warmer.setCursor(0, 1);
  if(!adc.init()){
    Serial.println("ADS1115 not connected!");
    warmer.print("NO ADC!");
  }
  else
  {
    Serial.println("ADS1115 found!");
    adc.setVoltageRange_mV(ADS1115_RANGE_2048); //comment line/change parameter to change range
    adc.setCompareChannels(ADS1115_COMP_0_GND); //comment line/change parameter to change channel
    adc.setConvRate(ADS1115_128_SPS);
    adc.setMeasureMode(ADS1115_CONTINUOUS);
    warmer.print("ADC OK!");
  }
  delay(500);
  warmer.clear();
  warmer.disp_setmode();
}

void loop()
{
  char valuetext[10];
  uint16_t frame = millis() / (1000/framerate);
  uint16_t second = warmer.countdown_target - (uint16_t)(millis() / 1000);

//タイマーが0になったらヒーターを止めて終了モードへ
  if (second == 0)
  {
    warmer.stop_heater();
    warmer.disp_finishmode();
  }

//ヒートモード中にセレクトボタンが押されたらヒーターを止めて設定モードへ
  else if (warmer.readButtons() == SELECT ||warmer.readButtons() == RIGHT)
  {
    warmer.stop_heater();
    warmer.mode = SET_FR_TEMP;
    sound(SOUND_PIN,100,2);
    warmer.disp_setmode();
  }

//ヒートモード　ターゲット温度にPID制御
  else if (warmer.mode == HEAT_MODE)
  {
    uint8_t min = second / 60;
    second = second % 60;

    for (int i = warmer.HEATER_FR; i <= warmer.HEATER_RL; i++)
    {
      ADS1115_MUX channel;
      switch (i)
      {
      case warmer.HEATER_FR:
        channel = ADS1115_COMP_0_GND;
        break;
      case warmer.HEATER_FL:
        channel = ADS1115_COMP_1_GND;
        break;
      case warmer.HEATER_RL:
        channel = ADS1115_COMP_2_GND;
        break;
      case warmer.HEATER_RR:
        channel = ADS1115_COMP_3_GND;
        break;      
      default:
        break;
      }
      float voltage=readChannel(channel);
      if(voltage==0)
      {
        warmer.stop_heater();
        Serial.println("No thermistor data");
        warmer.disp_errormode();
      }

      warmer.temp[i] = voltage_to_temp(voltage);
      if (warmer.temp[i] < 0)
        warmer.temp[i] = 0;

      //速度型PID制御
      float dt = 1.0 / warmer.sample_rate;
      if (i <= warmer.HEATER_FL)
      {
        warmer.dtemp[i] = ((float)warmer.target[0] - warmer.temp[i]) / dt; //偏差の微分 FR
      }
      else
      {
        warmer.dtemp[i] = ((float)warmer.target[1] - warmer.temp[i]) / dt; //偏差の微分 RR
      }
      P = warmer.dtemp[i] - warmer.pdtemp[i];
      warmer.pdtemp[i] = warmer.dtemp[i];
      I = warmer.dtemp[i] * dt;
      D = (P - warmer.preP[i]) / dt;
      warmer.preP[i] = P;

      warmer.duty[i] += (Kp * P + Ki * I + Kd * D) * dt;

      //dutyを0-255にクリップ
      if (warmer.duty[i] < 0 || warmer.temp[i]==0)
        warmer.duty[i] = 0;
      else if(warmer.duty[i]>255)
        warmer.duty[i]=255;
      uint32_t duty_out=(uint32_t)warmer.duty[i];

      analogWrite(warmer.heater_pin[i], duty_out);

      //シリアルでの監視用
      if (i <= warmer.HEATER_FL)
        Serial.print("F");
      else
        Serial.print("R");
      if (i % 2 == 0)
        Serial.print("R: ");
      else
        Serial.print("L:  ");
      //Serial.print(voltage,5);
      dtostrf(warmer.temp[i], 5, 2, valuetext);
      Serial.printf("%sC ", valuetext);

      Serial.printf("duty%3lu%% ",(uint32_t)(duty_out/2.55));
      dtostrf(warmer.duty[i], 5, 1, valuetext);
      Serial.printf("%s  ", valuetext);
    }
    Serial.printf("%2dfps\n", warmer.sample_rate);

    //PID計算用のADCサンプルレート取得
    if (second != lastsecond)
    {
      lastsecond = second;
      warmer.sample_rate = sps;
      sps = 0;
    }
    else
    {
      sps++;
    }

    //frameカウントが変わった時だけlcd更新
    if (frame != lastframe)
    {
      warmer.disp_heatmode(min, (uint8_t)second);
      lastframe = frame;
    }
  }
  else
  {
    warmer.stop_heater();
    Serial.println("Some error!");
    warmer.disp_errormode();
  }
}