//
// Created by 22627 on 2023/5/7.
//
#include "Common.h"
#include <Windows.h>
#include "Inc/PidController.h"

int main() {
    //基础pid
    PidController::BaseFactors_t pid1 = {0.3, 0.5, 0, 100, 200, 80, 1};
    PidController::BaseFactors_t pid2 = {0.2, 0.2, 0, 100, 200, 80, 1};
    PidController::BaseFactors_t pid3 = {0.1, 0.5, 0, 100, 200, 80, 1};
    std::vector<PidController::BaseFactors_t> pid={pid1,pid2,pid3};
    //分段pid
    std::vector<PidController::Segment_t> seg={{2000,20},{20,5},{5,-2000}};
    PidController::SegmentPidController spid(3,PidController::PID_POSITION);

    spid.PidInit(pid,seg);
    std::vector<fp32> out = {20,10,5};
    spid.advanced_factors_.SegmentForwardFeedInit(seg,out);
    spid.advanced_factors_.trapezium_integral_.flag =1;

    float aim=100;
    float ref=30;

    for(int i=0;i<20;i++)
    {
        ref = spid.PidSegmentCalc(aim,ref);
        printf("ref:%f\r\n",ref);
        Sleep(500);
    }

    printf("good!");

    return 0;
}