//
// Created by 22627 on 2023/5/7.
//
#include "Common.h"
#include <Windows.h>
#include "Inc/PidController.h"


#include <iostream>

// 打印Hello World
void printHelloWorld() {
    std::cout << "Hello World" << std::endl;
}

// 判断是否为闰年
bool isLeapYear(int year) {
    if ((year % 4 == 0 && year % 100 != 0) || year % 400 == 0) {
        return true; // 是闰年
    } else {
        return false; // 不是闰年
    }
}

// 判断是否为质数
bool isPrime(int num) {
    if (num <= 1) {
        return false; // 不是质数
    }

    for (int i = 2; i * i <= num; i++) {
        if (num % i == 0) {
            return false; // 不是质数
        }
    }

    return true; // 是质数
}

// 计算由a, b, c, d组合而成的无符号整数x
unsigned int calculateX(unsigned char a, unsigned char b, unsigned char c, unsigned char d) {
    unsigned int x = (static_cast<unsigned int>(a) << 24) | (static_cast<unsigned int>(b) << 16) |
                     (static_cast<unsigned int>(c) << 8) | static_cast<unsigned int>(d);
    return x;
}

// 将a的低m位取反，高(32-m)位置0
unsigned int reverseBits(unsigned int a, int m) {
    unsigned int mask = (1u << m) - 1u;
    return (~a & mask);
}

int main() {
//    //基础pid
//    TraditionalController::BaseFactors_t pid1 = {0.3, 0.5, 0, 100, 200, 80, 1};
//    TraditionalController::BaseFactors_t pid2 = {0.2, 0.2, 0, 100, 200, 80, 1};
//    TraditionalController::BaseFactors_t pid3 = {0.1, 0.5, 0, 100, 200, 80, 1};
//    std::vector<TraditionalController::BaseFactors_t> pid={pid1,pid2,pid3};
//    //分段pid
//    std::vector<TraditionalController::Segment_t> seg={{2000,20},{20,5},{5,-2000}};
//    TraditionalController::SegmentPidController spid(3,TraditionalController::PID_POSITION);
//
//    spid.PidInit(pid,seg);
//    std::vector<fp32> out = {20,10,5};
//    spid.advanced_factors_.SegmentForwardFeedInit(seg,out);
//
//    float aim=100;
//    float ref=30;
//
//    for(int i=0;i<20;i++)
//    {
//        ref = spid.PidSegmentCalc(aim,ref);
//        printf("ref:%f\r\n",ref);
//        Sleep(500);
//    }
//
//    printf("good!");

    // 1. 打印Hello World
    printHelloWorld();

    // 2. 从键盘输入一个年份，判断是否为闰年
    int year;
    printf("give me a year: ");
    scanf("%d", &year);

    if (isLeapYear(year)) {
        printf("yes run nian\n");
    } else {
        printf("no run nian\n");
    }

    // 3. 从键盘输入一个数，判断是否为质数
    int num;
    printf("input a int num:");
    scanf("%d", &num);

    if (isPrime(num)) {
        printf("yes zhishu\n");
    } else {
        printf("no zhishu\n");
    }

    // 4. 由a, b, c, d组合而成的无符号整数x
    unsigned char a, b, c, d;
    printf("input a four unsigned char data:");
    scanf("%hhu %hhu %hhu %hhu", &a, &b, &c, &d);

    unsigned int x = calculateX(a, b, c, d);
    printf("x: %u\n", x);

    // 5. 将a的低m位取反，高(32-m)位置0
    unsigned int result;
    unsigned int aaa;
    unsigned int m;
    printf("in put set and m is : ");
    scanf("%u %u",&aaa, &m);

    result = reverseBits(aaa, m);
    printf("calc is: %u\n", result);

    return 0;

    return 0;
}