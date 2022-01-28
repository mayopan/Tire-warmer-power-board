#include <TireWarmer.h>

void WarmerDisp::init(byte heater_FR_pin, byte heater_FL_pin, byte heater_RR_pin, byte heater_RL_pin, byte soundPin)
{
    setMCPType(LTI_TYPE_MCP23017);
    // set up the LCD's number of rows and columns:
    begin(16, 2);
    clearWriteError();
    Serial.println("lcd.begin done");
    // Print a message to the LCD.
    setBacklight(0x00);
    clear();
    home();
    print("Tire Warmer v1.0");
    _soundpin = soundPin;
    heater_pin[HEATER_FR] = heater_FR_pin;
    heater_pin[HEATER_FL] = heater_FL_pin;
    heater_pin[HEATER_RR] = heater_RR_pin;
    heater_pin[HEATER_RL] = heater_RL_pin;
    for (int i = HEATER_FR; i <= HEATER_RL; i++)
    {
        pinMode(i, OUTPUT);
    }
    stop_heater();
    sound(_soundpin, 200);
    mode = SET_FR_TEMP;
}

void WarmerDisp::stop_heater()
{
    for (int i = HEATER_FR; i <= HEATER_RL; i++)
    {
        analogWrite(heater_pin[i], 0);
    }
}

uint8_t WarmerDisp::getkey()
{
    int button = 0;
    while (!button)
    {
        button = readButtons();
    }
    if (mode == FINISH_MODE || mode == ERROR_MODE)
    {
        mode = SET_FR_TEMP;
        setBacklight(0);
        clear();
        sound(_soundpin, 50, 2);
    } // In ERROR/FINISH_MODE, any button input changes mode to setting mode (SET_FR_TEMP)
    else
    {
        switch (button)
        {
        case UP:
            if ((mode == SET_FR_TEMP || mode == SET_RR_TEMP) && target[mode - 1] < MAX_SET_TEMP)
                target[mode - 1] += 5;
            else if (mode == SET_TIMER)
                target[mode - 1]++;
            else
            {
            }
            break;
        case DOWN:
            if ((mode == SET_FR_TEMP || mode == SET_RR_TEMP) && target[mode - 1] > MIN_SET_TEMP)
                target[mode - 1] -= 5;
            else if (mode == SET_TIMER && target[mode - 1] > 1)
                target[mode - 1]--;
            break;
        case RIGHT:
            //if (mode != HEAT_MODE)
                mode++;
            if (mode > FINISH_MODE)
                mode = SET_FR_TEMP;
            break;
        case LEFT:
            if (mode != HEAT_MODE)
            {
                mode--;
                if (mode == 0)
                    mode = CONFIRM;
            }
            break;
        case SELECT:
            mode++;
            break;
        default:
            break;
        }
        sound(_soundpin, 30);
    }
    delay(300);
    return button;
}

void WarmerDisp::disp_setmode()
{
    Serial.println("in setup mode");
    pid_init();
    while (mode < HEAT_MODE)
    {
        if (mode != CONFIRM)
        {
            clear();
            home();
            print(ITEM_TEXT[mode - 1]);
            printf("%2d", target[mode - 1]);
            setCursor(0, 1);
            print(BUTTON_ITEM_TEXT);
            setBacklight(0);
        }
        else
        {

            clear();
            home();
            printf("TEMP FR %2d RR %2d", target[0], target[1]);
            setCursor(0, 1);
            printf("TIMER %2d min OK?", target[2]);
        }
        getkey();
    }
    countdown_target = millis() / 1000 + (uint16_t)target[2] * 60;
    setBacklight(0x1); //RED
    clear();
}

void WarmerDisp::disp_heatmode(uint8_t min, uint8_t second)
{
    /*    float resist = 0;

    for (int i = 0; i < 4; i++)
    {
        double voltage = (double)adcvalue[i] / (1 << 23) * 5000;
        double _resist = (INPUT_MVOLT / voltage - 1) * VOLTDEVRESIST / 1000; //[kΩ]
        temp[i] = CONSTB / log((float)_resist * exp(CONSTB / (THERMISTER_NOM_TEMP + ZERO_DEG_IN_KELVIN)) / THERMISTER_NOM_RESIST) - ZERO_DEG_IN_KELVIN;
        if (i == 1)
            resist = (float)_resist;
    }
    */
    home();
    printf("TIME RF%2.1fR%2.1f", temp[HEATER_FR], temp[HEATER_RR]);
    setCursor(0, 1);
    printf("%2u:%02uLF%2.1fR%2.1f", min, second, temp[HEATER_FL], temp[HEATER_RL]);
}

void WarmerDisp::disp_finishmode()
{
    sound(_soundpin, 3000);
    mode = FINISH_MODE;
    setBacklight(0x4); //GREEN
    home();
    printf("HEAT | FR%2.0f RR%2.0f", temp[HEATER_FR], temp[HEATER_RR]);
    setCursor(0, 1);
    printf("FINISH FL%2.0f RL%2.0f", temp[HEATER_FL], temp[HEATER_RL]);
    getkey();
    disp_setmode();
}

void WarmerDisp::disp_errormode()
{
    sound(_soundpin, 50, 10);
    mode = ERROR_MODE;
    setBacklight(0x5); //YELLOW
    home();
    print("THERMO BREAK!");
    setCursor(0, 1);
    print("STOPPED HEATING!");
    getkey();
    disp_setmode();
}

void WarmerDisp::pid_init()
{
    for (int i = 0; i < 4; i++)
    {
        duty[i] = 0;
        dtemp[i] = 0;
        pdtemp[i] = 0;
        preP[i] = 0;
    }
}

float voltage_to_temp(float voltage)
{
    double _resist = ((INPUT_MVOLT / 1000 / voltage - 1) * MEASURE_RESIST - VOLTAGE_DIVIDER_RESIST) / 1000; //[kΩ]
    return CONSTB / log((float)_resist * exp(CONSTB / (THERMISTER_NOM_TEMP + ZERO_DEG_IN_KELVIN)) / THERMISTER_NOM_RESIST) - ZERO_DEG_IN_KELVIN;
}

void sound(byte pin, uint16_t len, uint16_t num)
{
    for (uint16_t i = 0; i < num; i++)
    {
        digitalWrite(pin, HIGH);
        delay(len);
        digitalWrite(pin, LOW);
        delay(len);
    }
}
void sound(byte pin, uint16_t len)
{
    sound(pin, len, 1);
}