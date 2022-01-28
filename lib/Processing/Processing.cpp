#include <Processing.h>

void Processing::send_data()
{
    Serial.write(data,sizeof(data));
}

void Processing::reset()
{
    count=0;
}

void Processing::print()
{
    decode();
    Serial.println(data_in_text);
}

uint8_t Processing::encode(int8_t _data)
{
    data[count]='e';
    data[count+1]=_data;
    count+=2;
    return 2;
}

uint8_t Processing::encode(int16_t _data)
{
    data[count]='s';
    data[count+1]=_data>>8;
    data[count+2]=_data&0xff;
    count+=3;
    return 3;
}
uint8_t Processing::encode(int32_t _data)
{
    data[count]='t';
    for(int i=0;i<4;i++)
    {
        data[count+i+1]=(_data>>((3-i)*8))&0xff;
    }
    count+=5;
    return 5;
}

uint8_t Processing::encode(uint8_t _data)
{
    data[count]='E';
    data[count+1]=_data;
    count+=2;
    return 2;
}

uint8_t Processing::encode(uint16_t _data)
{
    data[count]='S';
    data[count+1]=_data>>8;
    data[count+2]=_data&0xff;
    count+=3;
    return 3;
}

uint8_t Processing::encode(uint32_t _data)
{
    data[count]='T';
    for(int i=0;i<4;i++)
    {
        data[count+i+1]=(_data>>((3-i)*8))&0xff;
    }
    count+=5;
    return 5;
}

uint8_t Processing::encode(float _data)
{
    data[count]='f';
    data[count+1]=(int16_t)_data>>8;
    data[count+2]=(int16_t)_data&0xff;
    count+=3;
    return 3;
}

uint8_t Processing::encode(double _data)
{
    data[count]='d';
    for(int i=0;i<8;i++)
    {
        data[count+i+1]=((int32_t)_data>>((3-i)*8))&0xff;
    }
    count+=9;
    return 9;
}

void Processing::decode()
{
    int i=0;
    while(count>0)
    {
        switch (data[i])
        {
        case 'e':
            /* code */
            data_in_text+=(int8_t)data[i+1];
            data_in_text+=',';
            i+=2;
            count-=2;
            break;
        case 's':
            /* code */
            int16_t number16;
            number16=((int16_t)data[i+1]<<8)|(int16_t)data[i+2];
            data_in_text+=number16;
            data_in_text+=',';
            i+=3;
            count-=3;
            break;
        case 't':
            /* code */
            int32_t number32=0x00000000;
            for(int j=0;j<4;j++)
            {
                number32|=(int32_t)data[i+j+1]<<(8*(3-j));
            }
            data_in_text+=number32;
            data_in_text+=',';
            i+=5;
            count-=5;
            break;
/*        case 'E':
            i+=2;
            count-=2;
            break;
        case 'S':
            i+=3;
            count-=3;
            break;
        case 'T':
            i+=5;
            count-=5;
            break; 
        case 'f':
            i+=3;
            count-=3;
            break;
        case 'd':
            i+=5;
            count-=5;
            break;     
        default:
            break;*/
        }

    }
}