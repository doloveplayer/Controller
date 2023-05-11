//
// Created by 22627 on 2023/5/1.
//
#include "Common.h"

template <typename T>
T linear_map(T _value, T _in_min, T _in_max, T _out_min, T _out_max) {
    T out_value = (_value - _in_min) / (_in_max - _in_min) * (_out_max - _out_min) + _out_min;
    if (_out_min < _out_max) {
        if (out_value < _out_min) {
            out_value = _out_min;
        }
        if (out_value > _out_max) {
            out_value = _out_max;
        }
    } else {
        if (out_value > _out_min) {
            out_value = _out_min;
        }
        if (out_value < _out_max) {
            out_value = _out_max;
        }
    }
    return out_value;
}