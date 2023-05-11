//
// Created by 22627 on 2023/5/1.
//
#ifndef CONTROLLER_PIDCONTROLLER_H
#define CONTROLLER_PIDCONTROLLER_H


#include "Common.h"

#include <vector>

namespace PidController {

    enum PidMode_e {
        PID_POSITION = 0,//位置式PID
        PID_DELTA//增量试
    };

    struct BaseFactors_t {
        //PID 三参数
        fp32 kp;
        fp32 ki;
        fp32 kd;

        //限幅
        fp32 max_out;  //最大输出
        fp32 max_i_out; //最大积分输出
        fp32 band_i;  //积分分离区间
        fp32 dead_band;//控制死区
    };

    struct OutPut_t {
        fp32 out;
        fp32 p_out;
        fp32 i_out;
        fp32 d_out;
        fp32 d_buf[3];  //微分项 0最新 1上一次 2上上次
        fp32 error[3]; //误差项 0最新 1上一次 2上上次
    };

    struct InPut_t {
        fp32 set;//设定值
        fp32 ref;//参考值
    };

    struct AdvancedFactors_t {
        /*前馈控制*/
        struct FeedForwardFactors_t {
            float kf;//前馈的一次系数
            float dead_band;//死区
        };
        /*微分先行*/
        /*梯形积分*/
        /*变速积分*/
    };

    struct Segment_t//分段的区间
    {
        float up_segment;
        float down_segment;
    };

    class SimplePidController {
    public:
        SimplePidController(PidMode_e _pid_mode) {
            //未调用初始化函数 使用默认的参数
            this->output_.error[2] = this->output_.error[1] = this->output_.error[0] = 0;
            this->input_.ref = this->input_.set = 0;
            this->factors_ = {0, 0, 0, 0, 0, 0, 0};
            this->pid_mode_ = _pid_mode;
        }

        SimplePidController() {};

        void PidInit(BaseFactors_t &_bfs);

        void PidSwitch(BaseFactors_t &_bfs);

        void PidModeSwitch(PidMode_e _pid_mode);

        fp32 PidCalc(fp32 _set, fp32 _ref);

        void PidClear();

    public:
        OutPut_t output_;//输出的参数
        InPut_t input_;//输入变量

    protected:
        fp32 PidCalcPosition(fp32 &_set, fp32 &_ref);

        fp32 PidCalcDelta(fp32 &_set, fp32 &_ref);

    protected:
        BaseFactors_t factors_;//基本控制参数
        AdvancedFactors_t advanced_factors_;//高级控制参数
        PidMode_e pid_mode_;//模式
    };

    class SegmentPidController : public SimplePidController {
    public:
        SegmentPidController(uint8_t _num_segment, PidMode_e _pid_mode) {
            this->num_segments_ = _num_segment;
            this->limits_.resize(_num_segment);
            this->factors_.resize(_num_segment);
            for (uint8_t i = 0; i < _num_segment; i++) {
                this->factors_[i] = default_factors_;
                this->limits_[i] = default_limits_;
            }
            SimplePidController::pid_mode_ = _pid_mode;//模式选择
        }

        fp32 PidSegmentCalc(fp32 _set, fp32 _ref);//分段计算

        void PidInit(std::vector<BaseFactors_t> &bfs, std::vector<Segment_t> &seg_);
    protected:
        uint8_t num_segments_;  // 分段数量
        BaseFactors_t default_factors_ = {1.0, 0.0, 0.0, 1000, 1000, 1000, 1000};  // 默认的PID参数
        Segment_t default_limits_ = {0, 0};  // 默认的分段上下限
        std::vector<BaseFactors_t> factors_;  // 每一段对应的PID参数
        std::vector<Segment_t> limits_;  // 每一段的区间
    };


    class FuzzyPidController : public SimplePidController {
    public:
    };
}

#endif //CONTROLLER_PidCONTROLLER_H


