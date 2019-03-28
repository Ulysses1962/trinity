/**
    @author Sergey Rotanov aka Ulysses
    @date   March 2019
    @brief  Channel API implementation
*/
#include "channel.hpp"

template<class T> Channel<T>::Channel(uint16_t channel_id, uint8_t channel_capacity) {
    id = channel_id;
    capacity = channel_capacity;
    data.clear();
}

template<class T> auto Channel<T>::push(T item) -> bool {
    if (data.size() == capacity) return false;
    data.push_back(item);
    return true;
}

template<class T> auto Channel<T>::pop(T& response) -> bool {
    if (!data.empty()) {
        response = *data.back();
        data.pop_back();
        return true;
    }
    return false;
}

