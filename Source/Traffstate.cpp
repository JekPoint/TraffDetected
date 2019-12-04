//
// Created by rick on 25.05.15.
//

#include "Traffstate.h"
#include <sys/shm.h>
#include <sys/types.h>
#include <time.h>
#include <iostream>

int32_t gettickcount(){
    struct timespec tv;
    clock_gettime(CLOCK_MONOTONIC, &tv);
    return (int32_t) (tv.tv_sec*1000+tv.tv_nsec/1000000);
}

PedState::PedState(){
    this->_pedstateid = shmget(15001, 8, IPC_CREAT | 0666);
    this->_pedtimeid = shmget(15002, 32, IPC_CREAT | 0666);
    this->state = (u_int8_t *) shmat(this->_pedstateid,NULL,0);
    this->time = (int32_t *) shmat(this->_pedtimeid,NULL,0);

}


void PedState::setstate(int index, bool state)
{
    if (this->state[index]!=state){
        this->state[index]=(u_int8_t)state;
        this->time[index] = gettickcount();
    //std::cout << this->time[index] << std::endl;
    }
}

bool PedState::getstate(int index)
{
    return (bool)this->state[index];
}
