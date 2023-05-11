//
// Created by 22627 on 2023/5/1.
//
/*基础命名规则 自定义用驼峰 类内小写_ */
#ifndef CONTROLLER_PIDCONTROLLER_H
#define CONTROLLER_PIDCONTROLLER_H


#include "Common.h"

#include <vector>

namespace PidController {

    enum PidMode_e {
        PID_POSITION = 0,//位置式PID
        PID_DELTA//增量试
    };

    struct Basefactors_t {
        //PID 三参数
        fp32 Kp;
        fp32 Ki;
        fp32 Kd;

        //限幅
        fp32 MaxOut;  //最大输出
        fp32 MaxIout; //最大积分输出
        fp32 BandI;  //积分分离区间
        fp32 DeadBand;//控制死区
    };

    struct OutPut_t {
        fp32 Out;
        fp32 Pout;
        fp32 Iout;
        fp32 Dout;
        fp32 Dbuf[3];  //微分项 0最新 1上一次 2上上次
        fp32 error[3]; //误差项 0最新 1上一次 2上上次
    };

    struct InPut_t {
        fp32 Set;//设定值
        fp32 Ref;//参考值
    };

    struct AdvancedFactors_t {
        /*前馈控制*/
        struct FeedForwardFactors_t {
            float KF;//前馈的一次系数
            float DeadBand;//死区
        };
        /*微分先行*/
        /*梯形积分*/
        /*变速积分*/
    };

    struct Segment_t//分段的区间
    {
        float UpSegment;
        float DownSegment;
    };

    class SimplePidController {
    public:
        SimplePidController(PidMode_e pidmode) {
            //未调用初始化函数 使用默认的参数
            this->output_.error[2] = this->output_.error[1] = this->output_.error[0] = 0;
            this->input_.Ref = this->input_.Set = 0;
            this->factors_ = {0, 0, 0, 0, 0, 0, 0};
            this->pidmode_ = pidmode;
        }

        SimplePidController() {};

        void PidInit(Basefactors_t &bfs);
        void PidSwitch(Basefactors_t &bfs);

        fp32 PidCalc(fp32 Set, fp32 Ref);

        void PidKeeper();

        void PidClear();

        fp32 PidCalcPosition(fp32 &Set, fp32 &Ref);

        fp32 PidCalcDelta(fp32 &Set, fp32 &Ref);

        Basefactors_t factors_;//基本控制参数
        AdvancedFactors_t Advfactors_;//高级控制参数
        OutPut_t output_;//输出的参数
        InPut_t input_;//输入变量
        PidMode_e pidmode_;//模式
    };

    class SegmentPidController : public SimplePidController {
    public:
        SegmentPidController(uint8_t numsegment, PidMode_e pidmode) {
            this->Num_segments_ = numsegment;
            this->limits_.resize(numsegment);
            this->factors_.resize(numsegment);
            for (uint8_t i = 0; i < numsegment; i++) {
                this->factors_[i] = default_factors_;
                this->limits_[i] = default_limits_;
            }
            SimplePidController::pidmode_ = pidmode;//模式选择
        }

        fp32 PidSegmentCalc(fp32 Set, fp32 Ref);//分段计算

        void PidInit(std::vector<Basefactors_t> &bfs, std::vector<Segment_t> &seg_);


        uint8_t Num_segments_;  // 分段数量
        Basefactors_t default_factors_ = {1.0, 0.0, 0.0, 1000, 1000, 1000, 1000};  // 默认的PID参数
        Segment_t default_limits_ = {0, 0};  // 默认的分段上下限
        std::vector<Basefactors_t> factors_;  // 每一段对应的PID参数
        std::vector<Segment_t> limits_;  // 每一段的区间
    };


    class FuzzyPidController : public SimplePidController {
    public:
    };
}

#endif //CONTROLLER_PidCONTROLLER_H


