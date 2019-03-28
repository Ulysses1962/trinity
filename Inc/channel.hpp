/**
    @author Sergey Rotanov aka Ulysses
    @date   March 2019
    @brief  Channel API definitions
*/
#ifndef __CHANNEL_H__
#define __CHANNEL_H__

#include "stdint.h"
#include "vector"

using namespace std;

template<class T> class Channel {
    uint16_t id;
    uint8_t  capacity;

    vector<T> data;
    
public:
    Channel(uint16_t channel_id, uint8_t channel_capacity);

    auto push(const T item) -> bool;
    auto pop(T& response) -> bool; 
};

#endif
