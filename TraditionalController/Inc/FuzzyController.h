//
// Created by 22627 on 2023/5/30.
//

#ifndef CONTROLLER_FUZZYCONTROLLER_CPP
#define CONTROLLER_FUZZYCONTROLLER_H

#include "Common.h"
#include <vector>
#include <map>
#include "math.h"

#include "Inc/PidController.h"

namespace TraditionalController {
    /*********隶属度函数的选择*********/
#define MEMBERSHIP_STEP 1.0f //隶属度步长 步长越大稳定性越好 越吃算力 越小就反之 不能小于 0.5
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
    /****默认的误差以及误差变化率范围****/
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

    typedef struct {
        /*pid参数的范围*/
        Segment_t kp_range_;
        Segment_t ki_range_;
        Segment_t kd_range_;
        /*误差与误差变化率的范围*/
        Segment_t e_range_;
        Segment_t er_range_;
    } FuzzyFactorRange_t;

    class FuzzyPidController : public SimplePidController {
    public:
        FuzzyPidController(PidMode_e _pid_mode) : SimplePidController(_pid_mode), e_{0.0, 0.0}, er_(0.0), mapped_e(0.0),
                                                  mapped_er(0.0),delat_kp_(0.0),delat_kd_(0.0),delat_ki_(0.0), pid_mapping_factor_(1){
            this->fuzzy_factor_range_.e_range_ = {InEMax, InEMin};
            this->fuzzy_factor_range_.er_range_ = {InERMax, InERMin};

#if MEMBERSHIPFUNCTION == triangular
            //三角隶属度函数必定对应论域中两个元素
//            this->e_index_.resize(2, 0);
//            this->e_memberShip_.resize(2, 0.0);
//            this->er_index_.resize(2, 0);
//            this->er_memberShip_.resize(2, 0.0);
#elif MEMBERSHIPFUNCTION == trapezoidal
            //梯形隶属度函数必定对应论域中两个元素
            e_index.resize(2);
            er_index.resize(2);
#elif MEMBERSHIPFUNCTION==gaussian
            //高斯隶属度函数必定对应论域中四个元素
            e_index.resize(4);
            er_index.resize(4);
#endif
        }

        /**
         * @brief          模糊PID类初始化函数
         * @param[in]      std::vector<BaseFactors_t> &bfs: PID控制参数
         * @param[in]
         * @retval         none
         */
        void PidInit(BaseFactors_t &_bfs, FuzzyFactorRange_t &_fuzzy_factor_range);

        void FuzzyPIDController(fp32 _e, fp32 _er);

        fp32 FuzzyPIDCalc(fp32 _set, fp32 _ref);

        /*pid的模糊增量*/
        fp32 delat_kp_;
        fp32 delat_ki_;
        fp32 delat_kd_;

        fp32 pid_mapping_factor_;

        FuzzyFactorRange_t fuzzy_factor_range_;

        fp32 e_[2];//误差
        fp32 er_;//误差变化率
        fp32 mapped_e;//映射到论域的误差
        fp32 mapped_er;//映射到论域的误差变化率

        std::map<int8_t,fp32> e_membership_index_;
        std::vector<fp32> e_memberShip_;//误差的隶属度
        std::vector<int8_t> e_index_;//误差隶属度对应位置
        std::map<int8_t,fp32> er_membership_index_;
        std::vector<fp32> er_memberShip_;
        std::vector<int8_t> er_index_;
    };
}

#endif //CONTROLLER_FUZZYCONTROLLER_CPP
