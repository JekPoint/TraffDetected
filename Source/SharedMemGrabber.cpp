//
// Created by rick on 14.04.15.
//

#include "SharedMemGrabber.h"
#include <sys/shm.h>
#include <string.h>
#include <iostream>


SharedMemGrabber::SharedMemGrabber(int width, int height, int memid) {
    this->_width = width;
    this->_height = height;
    this->_key = memid;
    this->_key_meta = this->_key +1;
    this->_shmid = shmget(this->_key, this->getBufferSize(), IPC_CREAT | 0666);
    this->_shmid_meta = shmget(this->_key_meta, sizeof(grabber_meta), IPC_CREAT | 0666);
    this->videobuf = (u_int8_t *) shmat(this->_shmid,NULL,0);
    this->frame_meta = (struct grabber_meta *) shmat(this->_shmid_meta,NULL,0);
    this->last_frame_videobuf = new u_int8_t[this->getBufferSize()];
    this->last_frame_meta = * this->frame_meta;
}

inline size_t SharedMemGrabber::getBufferSize() {
    return this->_height*this->_width*2;
}


bool SharedMemGrabber::getNextFrame() {
    if (this->frame_meta->NFrame>this->last_frame_meta.NFrame){
        this->last_frame_meta = * this->frame_meta;
        memcpy(this->last_frame_videobuf, this->videobuf, this->getBufferSize());
        return true;
    }
    return false;
}
