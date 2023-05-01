//
// Created by 22627 on 2023/5/1.
//

#ifndef CONTROLLER_COMMON_H
#define CONTROLLER_COMMON_H

#define min(a, b) ((a)<(b)?(a):(b))
#define max(a, b) ((a)>(b)?(a):(b))
#define ABS(x)  (((x)>0)?(x):-(x))//abs(x) is define in stdlib.h
#define constrain(amt, low, high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define round(x)     ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))
#define radians(deg) ((deg)*DEG_TO_RAD)
#define degrees(rad) ((rad)*RAD_TO_DEG)
#define sq(x)        ((x)*(x))

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
T linear_map(T value, T in_min, T in_max, T out_min, T out_max)


#endif //CONTROLLER_COMMON_H
