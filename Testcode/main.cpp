//
// Created by 22627 on 2023/5/7.
//
#include "Common.h"
#include <Windows.h>
#include "Inc/PidController.h"
#include "Inc/FuzzyController.h"
#include <chrono>

#include <iostream>
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
                                                     {50, -50},
                                                     {50, -50}
    };
    TraditionalController::FuzzyPidController Fpid(TraditionalController::PID_POSITION);
    Fpid.PidInit(pid1, ffr);

    float aim = 100;
    float ref = 50;

    for (int i = 0; i < 20; i++) {
        std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
        ref = Fpid.FuzzyPIDCalc(aim, ref);
//        ref = spid.PidSegmentCalc(aim, ref);
        std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::ratio<1, 1>> duration_s(t2 - t1);
        std::cout << duration_s.count() << "seconds" << std::endl;
        printf("ref:%f\r\n***********\r\n", ref);
        Sleep(500);
    }

    printf("good!");
    return 0;
}