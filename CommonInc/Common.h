//
// Created by 22627 on 2023/5/1.
//
#include "stdio.h"
//#include "math.h"

#ifndef COMMON_H
#define COMMON_H

#define ABS(x)  (((x)>0)?(x):-(x))
#define constrain(amt, low, high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define INRANGE(value, min, max) ((value) >= (min) && (value) <= (max) ? 1 : 0)
#define LIMITMAX(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

#define LIMITBAND(input, max, min)   \
    {                                \
        if (input > max)             \
        {                            \
            input = max;             \
        }                            \
        else if (input < min)        \
        {                            \
            input = min;             \
        }                            \
    }

typedef float fp32;
typedef signed char int8_t;
typedef signed short int int16_t;
typedef signed int int32_t;
typedef unsigned char uint8_t;
typedef unsigned short int uint16_t;
typedef unsigned int uint32_t;

/**
  * @brief          线性映射函数
  * @param[in]      &value 要映射的值
	*				&in_min 输入的最小值
	*				&in_max	输入的最大值
	*				&out_min 映射区间的最小值
	*				&out_max 映射区间的最大值
  * @retval         none
  */
template<typename T>
T linear_map(T value, T in_min, T in_max, T out_min, T out_max);


#endif //COMMON_H
