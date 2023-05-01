//
// Created by 22627 on 2023/5/1.
//
#include "Common.h"

template<typename T>
T linear_map(T value, T in_min, T in_max, T out_min, T out_max) {
    T out_value = (value - in_min) / (in_max - in_min) * (out_max - out_min) + out_min;
    if (out_min < out_max) {
        if (out_value < out_min) {
            out_value = out_min;
        }
        if (out_value > out_max) {
            out_value = out_max;
        }
    } else {
        if (out_value > out_min) {
            out_value = out_min;
        }
        if (out_value < out_max) {
            out_value = out_max;
        }
    }
    return out_value;
}