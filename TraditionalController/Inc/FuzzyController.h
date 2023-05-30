//
// Created by 22627 on 2023/5/30.
//

#ifndef CONTROLLER_FUZZYCONTROLLER_CPP
#define CONTROLLER_FUZZYCONTROLLER_H

#include "Common.h"
#include <vector>
#include "math.h"

#include "Inc/PidController.h"

namespace TraditionalController {


    /*********隶属度函数的选择*********/
#define triangular 0//三角
#define trapezoidal 1//梯形
#define gaussian 2//高斯
#define MEMBERSHIPFUNCTION triangular
    /**********论域宏定义**********/
    static const fp32 NB = -3;
    static const fp32 NM = -2;
    static const fp32 NS = -1;
    static const fp32 ZO = 0;
    static const fp32 PS = 1;
    static const fp32 PM = 2;
    static const fp32 PB = 3;

    static const fp32 Domain[7] = {NB, NM, NS, ZO, PS, PM, PB};

    static const fp32 InEMin = -3.14f;
    static const fp32 InEMax = 3.14f;
    static const fp32 InERMin = -20.0f;
    static const fp32 InERMax = 20.0f;


    /***********Kp规则表***********/
    static const float KpRulesTable[7][7] =
            {
                    PB, PB, PM, PM, PS, ZO, ZO,
                    PB, PB, PM, PS, PS, ZO, NS,
                    PM, PM, PM, PS, ZO, NS, NS,
                    PM, PM, PS, ZO, NS, NM, NM,
                    PS, PS, ZO, NS, NS, NM, NM,
                    PS, ZO, NS, NM, NM, NM, NB,
                    ZO, ZO, NM, NM, NM, NB, NB
            };

    /***********Ki规则表***********/
    static const float KiRulesTable[7][7] =
            {
                    NB, NB, NM, NM, NS, ZO, ZO,
                    NB, NB, NM, NS, NS, ZO, ZO,
                    NB, NM, NS, NS, ZO, PS, PS,
                    NM, NM, NS, ZO, PS, PM, PM,
                    NM, NS, ZO, PS, PS, PM, PB,
                    ZO, ZO, PS, PS, PM, PB, PB,
                    ZO, ZO, PS, PM, PM, PB, PB
            };

    /***********Kd规则表***********/
    static const float KdRulesTable[7][7] =
            {
                    PS, NS, NB, NB, NB, NM, PS,
                    PS, NS, NB, NM, NM, NS, ZO,
                    ZO, NS, NM, NM, NS, NS, ZO,
                    ZO, NS, NS, NS, NS, NS, ZO,
                    ZO, ZO, ZO, ZO, ZO, ZO, ZO,
                    PB, NS, PS, PS, PS, PS, PB,
                    PB, PM, PM, PM, PS, PS, PB
            };

    class FuzzyPidController : public SimplePidController {
    public:
        void FuzzyPIDController(fp32 _e, fp32 _er);

        void FuzzyPIDCalc(fp32 _set, fp32 _ref);

        fp32 delat_kp_;
        fp32 delat_ki_;
        fp32 delat_kd_;
        fp32 mapped_e;
        fp32 mapped_er;
        std::vector<fp32> e_memberShip_;
        std::vector<fp32> er_memberShip_;
    };
}

#endif //CONTROLLER_FUZZYCONTROLLER_CPP
