//
// Created by 22627 on 2023/5/1.
//

#ifndef CONTROLLER_PIDCONTROLLER_H
#define CONTROLLER_PIDCONTROLLER_H

#include "Common.h"

namespace TraditionalController {

#define USESEGMENTPID 1 //是否使用分段pid控制器 如果使用则需要包含以下库

#if USESEGMENTPID

#include <tuple>
#include <vector>

#else
#endif

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
        fp32 out;
        fp32 Pout;
        fp32 Iout;
        fp32 Dout;
        fp32 Dbuf[3];  //微分项 0最新 1上一次 2上上次
        fp32 error[3]; //误差项 0最新 1上一次 2上上次
    };

    struct InPut_t {
        fp32 Set;//设定值
        fp32 ref;//参考值
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

    class BasePidController {
    public:
        Basefactors_t factors;
        OutPut_t outPut;
        InPut_t inPut;
    };

    class SimplePidController : public BasePidController {
    public:
        SimplePidController(Basefactors_t bfs);

        ~SimplePidController();

        fp32 PidCalc(fp32 Set, fp32 ref);

        void PidKeeper();

        void PidClear();
    };

#if USESEGMENTPID

    class SegmentPidController : public BasePidController {
    public:

        struct Segment_t//分段的区间
        {
            float UpSegment;
            float DownSegment;
        };

        // 可变参数模板实现
        template<typename... Args>
        SegmentPidController(Args... segments, Args... factors) {
            // 将可变参数保存到vector中
            factors_ = {factors...};
            segments_ = {segments...};
            // 分段数量即为参数数量除以segment
            NumSegments_ = sizeof...(segments) / sizeof(Segment_t);
            if(NumSegments_ != sizeof...(factors) / sizeof(Basefactors_t))
            {
                //错误处理函数
            }
            else
            {
                //正常
            }
        }

        std::vector<Basefactors_t> factors_;  // 每一段对应的PID参数
        std::vector<Segment_t> segments_;  // 每一段的区间
        int NumSegments_;  // 分段数量
        Basefactors_t default_params_ = {1.0, 0.0, 0.0, 1000, 1000, 1000, 1000};  // 默认的PID参数

    };

#else
#endif


    class FuzzyPidController : public BasePidController {
    public:
    };
}

#endif //CONTROLLER_PIDCONTROLLER_H


