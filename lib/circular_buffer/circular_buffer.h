#ifndef CIRCULAR_BUFFER_H
#define CIRCULAR_BUFFER_H

#include <Arduino.h>

const int lenght_buffer=50;

class CIRCULAR_BUFFER{
    public:
    CIRCULAR_BUFFER();
    void put(float data);
    float sum_buffer();

    private:
    float buffer[lenght_buffer]={0};
    int index=0;
};


#endif