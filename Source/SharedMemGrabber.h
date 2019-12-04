//
// Created by rick on 14.04.15.
//

#ifndef JPEG_COMPRESSOR_SHAREDMEMGRABBER_H
#define JPEG_COMPRESSOR_SHAREDMEMGRABBER_H


#include <stddef.h>
#include <sys/time.h>
#include <sys/types.h>

struct grabber_meta
{
    int NFrame;
    char Grabbed;
    char Reading;
    struct timeval timestamp;
};

struct grabber_meta_compressed
{
    int NFrame;
    char Grabbed;
    char Reading;
    struct timeval timestamp;
    int size;
};

class SharedMemGrabber {
public:
    u_int8_t * videobuf;
    u_int8_t * last_frame_videobuf;
    struct grabber_meta * frame_meta;
    struct grabber_meta last_frame_meta;
    SharedMemGrabber(int width, int height, int memid);
    size_t getBufferSize();
    bool getNextFrame();
private:
    int _width;
    int _height;
    int _key;
    int _key_meta;
    int _shmid;
    int _shmid_meta;
};


#endif //JPEG_COMPRESSOR_SHAREDMEMGRABBER_H
