#ifndef Processing_h
#define Processing_h


#include <Arduino.h>

class Processing
{
    public:
    void send_data();
    void reset();
    String data_in_text;
    uint8_t encode(int8_t _data);
    uint8_t encode(int16_t _data);
    uint8_t encode(int32_t _data);
    uint8_t encode(uint8_t _data);
    uint8_t encode(uint16_t _data);
    uint8_t encode(uint32_t _data);
    uint8_t encode(float _data);
    uint8_t encode(double _data);
    void print();

    private:
    char data[256];
    uint8_t count=0;
    char int8[2];//e
    char int16[3];//s
    char int32[5];//t
    char uint8[2];//E
    char uint16[3];//S
    char uint32[5];//T
    char _float[5];//f
    char _double[9];//d
    void decode();
};

#endif