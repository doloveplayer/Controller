//
// Created by 22627 on 2023/5/1.
//

#include "Inc/PidController.h"

namespace PidController {
    void SimplePidController::PidInit(BaseFactors_t &_bfs) {
        this->output_.error[2] = this->output_.error[1] = this->output_.error[0] = 0;
        this->input_.ref = this->input_.set = 0;
        this->factors_ = _bfs;
    }

    void SimplePidController::PidSwitch(BaseFactors_t &_bfs) {
        this->factors_ = _bfs;
    }

    fp32 SimplePidController::PidCalcPosition(fp32 &_set, fp32 &_ref) {
        this->output_.error[2] = this->output_.error[1];
        this->output_.error[1] = this->output_.error[0];
        this->input_.set = _set;
        this->input_.ref = _ref;

        //更新当前误差
        this->output_.error[0] = this->input_.set - this->input_.ref;
        if (ABS(this->output_.error[0]) < this->factors_.dead_band) {
            return this->output_.out;
        }
        //计算p
        this->output_.p_out = this->factors_.kp * this->output_.error[0];
        //计算i
        this->output_.i_out += this->factors_.ki * this->output_.error[0];
        LIMITMAX(this->output_.i_out, this->factors_.max_i_out);
        //计算d
        this->output_.d_buf[2] = this->output_.d_buf[1];
        this->output_.d_buf[1] = this->output_.d_buf[0];
        this->output_.d_buf[0] = (this->output_.error[0] - this->output_.error[1]);
        this->output_.d_out = this->factors_.kd * this->output_.d_buf[0];

        this->output_.out = this->output_.p_out + this->output_.i_out + this->output_.d_out;

        return this->output_.out;
    }

    fp32 SimplePidController::PidCalcDelta(fp32 &_set, fp32 &_ref) {
        this->output_.error[2] = this->output_.error[1];
        this->output_.error[1] = this->output_.error[0];
        this->input_.set = _set;
        this->input_.ref = _ref;

        //更新当前误差
        this->output_.error[0] = this->input_.set - this->input_.ref;
        if (ABS(this->output_.error[0]) < this->factors_.dead_band) {
            //误差小于死去时停止调用pid
            return this->output_.out;
        }
        //计算p
        this->output_.p_out = this->factors_.kp * (this->output_.error[0] - this->output_.error[1]);
        //计算i
        this->output_.i_out += this->factors_.ki * this->output_.error[0];
        //计算d
        this->output_.d_buf[2] = this->output_.d_buf[1];
        this->output_.d_buf[1] = this->output_.d_buf[0];
        this->output_.d_buf[0] = (this->output_.error[0] - 2.0f * this->output_.error[1] + this->output_.error[2]);
        this->output_.d_out = this->factors_.kd * this->output_.d_buf[0];

        this->output_.out += this->output_.p_out + this->output_.i_out + this->output_.d_out;

        return this->output_.out;
    }

    fp32 SimplePidController::PidCalc(fp32 _set, fp32 _ref) {
        switch (this->pid_mode_) {
            case PID_POSITION:
                return PidCalcPosition(_set, _ref);
            case PID_DELTA:
                return PidCalcDelta(_set, _ref);
            default:
                return 0;
        }
    }

    fp32 SegmentPidController::PidSegmentCalc(fp32 _set, fp32 _ref) {
        int index = 0;
        for (; index < this->num_segments_; index++) {
            if (INRANGE((_set - _ref), this->limits_[index].down_segment, this->limits_[index].up_segment)) {
                SimplePidController::PidSwitch(this->factors_[index]);//切换pid参数而不改变输入输出量
                return SimplePidController::PidCalc(_set, _ref);
            }
        }
        //如果不在范围内就使用默认参数
        {
            SimplePidController::PidSwitch(default_factors_);//切换pid参数而不改变输入输出量
            SimplePidController::PidCalc(_set, _ref);
            return SimplePidController::PidCalc(_set, _ref);
        };
    }

    void SegmentPidController::PidInit(std::vector<BaseFactors_t> &_bfs, std::vector<Segment_t> &_seg) {
        if (_bfs.size() != this->num_segments_ && _seg.size() != this->num_segments_) {
            //错误处理函数
            printf("error segment");
            return;
        } else {
            this->factors_ = _bfs;
            this->limits_ = _seg;
        }

    }
}




