#ifndef TireWarmer_h
#define TireWarmer_h

#ifndef LiquidTWI2_h
#include <LiquidTWI2.h>
#endif

#include <Arduino.h>

const uint8_t SET_FR_TEMP = 1;
const uint8_t SET_RR_TEMP = 2;
const uint8_t SET_TIMER = 3;
const uint8_t CONFIRM = 4;
const uint8_t HEAT_MODE = 5;
const uint8_t FINISH_MODE = 6;
const uint8_t ERROR_MODE = 6;
const uint8_t SELECT = 1;
const uint8_t LEFT = 16;
const uint8_t RIGHT = 2;
const uint8_t UP = 8;
const uint8_t DOWN = 4;
const String ITEM_TEXT[3] = {"1:FR TEMP SET ", "2:RR TEMP SET ", "3:TIMER SET  "};
const String BUTTON_ITEM_TEXT = "<> ITEM ^v VALUE";

const float CONSTB = 5000.0;//5300
const float MEASURE_RESIST = 12000; //measured 220 [Ω] resist
const float VOLTAGE_DIVIDER_RESIST = 7500;
const float ZERO_DEG_IN_KELVIN = 273.15;
const float INPUT_MVOLT = 3300.0;         //mV
const float THERMISTER_NOM_RESIST = 10.0; //[kΩ]
const float THERMISTER_NOM_TEMP = 25.0;   //[°C]
const uint8_t MAX_SET_TEMP = 90;
const uint8_t MIN_SET_TEMP = 20;

class WarmerDisp : public LiquidTWI2
{
public:
    WarmerDisp(uint8_t i2cAddr, uint8_t detectDevice = 0, uint8_t backlightInverted = 0) : LiquidTWI2(i2cAddr, detectDevice, backlightInverted)
    {
    }
    void init(byte heater_FR_pin, byte heater_FL_pin, byte heater_RR_pin, byte heater_RL_pin, byte sound_pin);
    uint8_t getkey();
    void disp_setmode();
    void disp_finishmode();
    void disp_errormode();
    void disp_heatmode(uint8_t min, uint8_t second);
    void pid_init();
    void stop_heater();
    uint8_t target[3] = {50, 50, 5};
    float temp[4] = {25, 25, 25, 25};
    uint16_t countdown_target = 0;
    uint8_t mode = 0;
    uint16_t sample_rate = 10;
    float duty[4];
    float dtemp[4];
    float pdtemp[4];
    float preP[4];
    byte heater_pin[4];
    enum
    {
        HEATER_FR,
        HEATER_FL,
        HEATER_RR,
        HEATER_RL
    };
    private:
    byte _soundpin;
};

/* Output float temperature in cecius degrees from adc int value */
float voltage_to_temp(float voltage);

void sound(byte pin, uint16_t len, uint16_t num);
void sound(byte pin, uint16_t len);

#endif