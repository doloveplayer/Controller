//
// Created by 22627 on 2023/5/7.
//
#include "Common.h"
#include "Inc/PidController.h"

int main() {
    PidController::Basefactors_t pid = {1, 1, 1, 1, 1, 1, 1};
    PidController::PidMode_e pe = PidController::PID_POSITION;
    PidController::SimplePidController spid(pid, pe);

    printf("good!");

    return 0;
}