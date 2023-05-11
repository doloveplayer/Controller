//
// Created by 22627 on 2023/5/7.
//
#include "Common.h"
#include <Windows.h>
#include "Inc/PidController.h"

int main() {
    PidController::BaseFactors_t pid1 = {0.3, 0.5, 0, 30, 200, 80, 1};
    PidController::BaseFactors_t pid2 = {0.2, 0.2, 0, 30, 200, 80, 1};
    PidController::BaseFactors_t pid3 = {0.1, 0.1, 0, 30, 200, 80, 1};
    std::vector<PidController::BaseFactors_t> pid={pid1,pid2,pid3};
    std::vector<PidController::Segment_t> seg={{2000,20},{20,5},{5,-2000}};
    PidController::PidMode_e pe = PidController::PID_POSITION;
    PidController::SegmentPidController spid(3,pe);
    spid.PidInit(pid,seg);

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