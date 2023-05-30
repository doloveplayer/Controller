//
// Created by 22627 on 2023/5/1.
//
#ifndef CONTROLLER_PIDCONTROLLER_H
#define CONTROLLER_PIDCONTROLLER_H


#include "Common.h"
#include <vector>

namespace TraditionalController {

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

    struct Segment_t//分段的区间
    {
        float up_segment;
        float down_segment;
    };

    class AdvancedFactors {
    public:
        /*前馈控制*/
        struct ForwardFeed_t {
            uint8_t flag;
            /*电机模型的前馈控制 参照华南虎*/
            fp32 last_in;//上一次的输入
            fp32 time;//采样周期（s）
            /*分段模型的前馈控制 暴力使用*/
            std::vector<Segment_t> ref_segment;//参考区间
            std::vector<fp32> out;//对应分段输出
            uint8_t segment_size;//分段数量
        };

        /**
         * @brief          模型前馈控制
         * @param[in]      fp32 _in: 输入
         * @retval         输出
         */
        fp32 ForwardFeed(fp32 _in);

        /**
         * @brief          分段前馈控制初始化
         * @param[in]      fp32 _in: 输入
         * @retval         输出
         */
        void SegmentForwardFeedInit(std::vector<Segment_t> &_ref_seg, std::vector<fp32> &_out);

        /**
         * @brief          分段前馈控制
         * @param[in]      fp32 _err: 当前参考值
         * @retval         输出
         */
        fp32 SegmentForwardFeed(fp32 _err);

        /*梯形积分*/
        struct TrapeziumIntegral_t {
            uint8_t flag;
        };
        /*变速积分*/
        struct VariableIntergral_t {
            uint8_t flag;
            fp32 max_intergral;
            fp32 min_intergral;
        };

        /**
        * @brief          变速积分初始化函数
        * @param[in]      fp32 _maxInterval: 积分区间上限
        * @param[in]      fp32 _minInterval: 积分区间下限
        * @retval         变速积分系数
        */
        void VariableIntergralInit(fp32 _max_interval, fp32 _min_interval);

        /**
        * @brief          变速积分系数计算函数
        * @param[in]      fp32& _max_interval: 积分区间上限
        * @param[in]      fp32& _min_interval: 积分区间下限
        * @param[in]      fp32& _error: 	本次误差
        * @retval         变速积分系数
        */
        fp32 VariableIntergralCoefficientCalc(fp32 &_max_interval, fp32 &_min_interval, fp32 &_error);

        ForwardFeed_t forward_feed_;
        TrapeziumIntegral_t trapezium_integral_;
        VariableIntergral_t variable_integral_;
    };

    class SimplePidController {
    public:
        /**
         * @brief          简单PID类构造函数
         * @param[in]      PidMode_e _pid_mode: PID类型
         * @retval         none
         */
        SimplePidController(PidMode_e _pid_mode) {
            //未调用初始化函数 使用默认的参数
            this->output_.error[2] = this->output_.error[1] = this->output_.error[0] = 0;
            this->input_.ref = this->input_.set = 0;
            this->factors_ = {0, 0, 0, 0, 0, 0, 0};
            this->pid_mode_ = _pid_mode;

            this->advanced_factors_.forward_feed_.flag = 0;
            this->advanced_factors_.trapezium_integral_.flag = 0;
            this->advanced_factors_.variable_integral_.flag = 0;
        }

        /**
         * @brief          简单PID类默认构造函数
         * @param[in]      none
         * @retval         none
         */
        SimplePidController() {
            this->advanced_factors_.forward_feed_.flag = 0;
            this->advanced_factors_.trapezium_integral_.flag = 0;
            this->advanced_factors_.variable_integral_.flag = 0;
        };

        /**
         * @brief          简单PID类初始化函数
         * @param[in]      BaseFactors_t &_bfs: PID控制参数
         * @retval         none
         */
        void PidInit(BaseFactors_t &_bfs);

        /**
         * @brief          简单PID类切换参数函数
         * @param[in]      BaseFactors_t &_bfs: PID控制参数
         * @retval         none
         */
        void PidSwitch(BaseFactors_t &_bfs);

        /**
         * @brief          简单PID类切换类型函数
         * @param[in]      PidMode_e _pid_mode: PID类型
         * @retval         none
         */
        void PidModeSwitch(PidMode_e _pid_mode);

        /**
         * @brief          简单PID类计算函数
         * @param[in]      fp32  _set: PID设定值
         * @param[in]      fp32  _ref: PID参考值
         * @retval         none
         */
        fp32 PidCalc(fp32 _set, fp32 _ref);

        /**
         * @brief          简单PID类清零函数
         * @param[in]      none
         * @retval         none
         */
        void PidClear();

    public:
        OutPut_t output_;//输出的参数
        InPut_t input_;//输入变量

        BaseFactors_t factors_;//基本控制参数
        AdvancedFactors advanced_factors_;//高级控制参数

    protected:
        fp32 PidCalcPosition(fp32 &_set, fp32 &_ref);

        fp32 PidCalcDelta(fp32 &_set, fp32 &_ref);

        PidMode_e pid_mode_;//模式
    };

    class SegmentPidController : public SimplePidController {
    public:
        /**
         * @brief          分段PID类构造函数
         * @param[in]      uint8_t _num_segment:分段数量
         * @param[in]      PidMode_e _pid_mode:pid模式
         * @retval         none
         */
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

        /**
         * @brief          分段PID类计算函数
         * @param[in]      fp32  _set: PID设定值
         * @param[in]      fp32  _ref: PID参考值
         * @retval         none
         */
        fp32 PidSegmentCalc(fp32 _set, fp32 _ref);//分段计算
        /**
         * @brief          分段PID类初始化函数
         * @param[in]      std::vector<BaseFactors_t> &bfs: PID控制参数
         * @param[in]      std::vector<Segment_t> &seg_: PID分段区间
         * @retval         none
         */
        void PidInit(std::vector<BaseFactors_t> &bfs, std::vector<Segment_t> &seg_);

    protected:
        uint8_t num_segments_;  // 分段数量
        BaseFactors_t default_factors_ = {1.0, 0.0, 0.0, 1000, 1000, 1000, 1000};  // 默认的PID参数
        Segment_t default_limits_ = {0, 0};  // 默认的分段上下限
        std::vector<BaseFactors_t> factors_;  // 每一段对应的PID参数
        std::vector<Segment_t> limits_;  // 每一段的区间
    };
}

#endif //CONTROLLER_PidCONTROLLER_H


