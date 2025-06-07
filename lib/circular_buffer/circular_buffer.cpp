#include "circular_buffer.h"

CIRCULAR_BUFFER::CIRCULAR_BUFFER(){
}

void CIRCULAR_BUFFER::put(float data){
    this->buffer[this->index]=data;
    this->index++;
    if(this->index>lenght_buffer){
        this->index=0;
    }
}

float CIRCULAR_BUFFER::sum_buffer(){
    float sum=0;
    for(int i=0; i<lenght_buffer; i++){
        sum+=this->buffer[i];
    }
    return sum;
}