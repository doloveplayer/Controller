//
// Created by 22627 on 2023/5/7.
//
#include "Common.h"
#include <Windows.h>
#include "Inc/PidController.h"

int main() {
    PidController::Basefactors_t pid = {0.5, 0.05, 0, 30, 40, 80, 1};
    PidController::PidMode_e pe = PidController::PID_POSITION;
    PidController::SimplePidController spid(pid, pe);

    float aim=100;
    float ref=30;
    printf("p: %f,i: %f,d:%f\r\n",spid.factors_.Kp,spid.factors_.Ki,spid.factors_.Kd);
    printf("mode:%d Maxout:%f,MaxIout:%f\r\n",spid.pidmode_,spid.factors_.MaxOut,spid.factors_.MaxIout);

    for(int i=0;i<10;i++)
    {
        ref = spid.PidCalc(aim,ref);
        printf("calc out :%f\r\n",spid.PidCalc(aim,ref));
        printf("ref:%f\r\n",ref);
        Sleep(500);
//        printf("set: %f,ref: %f\r\n",aim,ref);
    }

    printf("good!");

    return 0;
}