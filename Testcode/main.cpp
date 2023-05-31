//
// Created by 22627 on 2023/5/7.
//
#include "Common.h"
#include <Windows.h>
#include "Inc/PidController.h"
#include "Inc/FuzzyController.h"


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
    //基础pid
    TraditionalController::BaseFactors_t pid1 = {0.3, 0.5, 0, 200, 200, 80, 1};
    TraditionalController::BaseFactors_t pid2 = {0.2, 0.2, 0, 200, 200, 80, 1};
    TraditionalController::BaseFactors_t pid3 = {0.1, 0.5, 0, 200, 200, 80, 1};
    std::vector<TraditionalController::BaseFactors_t> pid = {pid1, pid2, pid3};
    //分段pid
    std::vector<TraditionalController::Segment_t> seg = {{2000, 20},
                                                         {20,   5},
                                                         {5,    -2000}};
    TraditionalController::SegmentPidController spid(3, TraditionalController::PID_POSITION);

    spid.PidInit(pid, seg);
    std::vector<fp32> out = {20, 10, 5};
    spid.advanced_factors_.SegmentForwardFeedInit(seg, out);

    TraditionalController::FuzzyFactorRange_t ffr = {{10, -6},
                                                     {10, -6},
                                                     {10, -6},
                                                     {50,-50},
                                                     {50,-50}
                                                     };
    TraditionalController::FuzzyPidController Fpid(TraditionalController::PID_POSITION);
    Fpid.PidInit(pid1,ffr);

    float aim = 10;
    float ref = 50;

    for (int i = 0; i < 20; i++) {
        ref = Fpid.FuzzyPIDCalc(aim, ref);
        printf("ref:%f\r\n***********\r\n", ref);
        Sleep(500);
    }

    printf("good!");
    return 0;
}