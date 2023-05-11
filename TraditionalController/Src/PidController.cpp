//
// Created by 22627 on 2023/5/1.
//

#include "Inc/PidController.h"

namespace PidController {
    void SimplePidController::PidInit(Basefactors_t &bfs) {
        this->output_.error[2] = this->output_.error[1] = this->output_.error[0] = 0;
        this->input_.Ref = this->input_.Set = 0;
        this->factors_ = bfs;
    }

    void SimplePidController::PidSwitch(Basefactors_t &bfs) {
        this->factors_ = bfs;
    }

    fp32 SimplePidController::PidCalcPosition(fp32 &Set, fp32 &Ref) {
        this->output_.error[2] = this->output_.error[1];
        this->output_.error[1] = this->output_.error[0];
        this->input_.Set = Set;
        this->input_.Ref = Ref;

        //更新当前误差
        this->output_.error[0] = this->input_.Set - this->input_.Ref;
        if (ABS(this->output_.error[0]) < this->factors_.DeadBand) {
            return this->output_.Out;
        }
        //计算p
        this->output_.Pout = this->factors_.Kp * this->output_.error[0];
        //计算i
        this->output_.Iout += this->factors_.Ki * this->output_.error[0];
        LIMITMAX(this->output_.Iout, this->factors_.MaxIout);
        //计算d
        this->output_.Dbuf[2] = this->output_.Dbuf[1];
        this->output_.Dbuf[1] = this->output_.Dbuf[0];
        this->output_.Dbuf[0] = (this->output_.error[0] - this->output_.error[1]);
        this->output_.Dout = this->factors_.Kd * this->output_.Dbuf[0];

        this->output_.Out = this->output_.Pout + this->output_.Iout + this->output_.Dout;

        return this->output_.Out;
    }

    fp32 SimplePidController::PidCalcDelta(fp32 &Set, fp32 &Ref) {
        this->output_.error[2] = this->output_.error[1];
        this->output_.error[1] = this->output_.error[0];
        this->input_.Set = Set;
        this->input_.Ref = Ref;

        //更新当前误差
        this->output_.error[0] = this->input_.Set - this->input_.Ref;
        if (ABS(this->output_.error[0]) < this->factors_.DeadBand) {
            //误差小于死去时停止调用pid
            return this->output_.Out;
        }
        //计算p
        this->output_.Pout = this->factors_.Kp * (this->output_.error[0] - this->output_.error[1]);
        //计算i
        this->output_.Iout += this->factors_.Ki * this->output_.error[0];
        //计算d
        this->output_.Dbuf[2] = this->output_.Dbuf[1];
        this->output_.Dbuf[1] = this->output_.Dbuf[0];
        this->output_.Dbuf[0] = (this->output_.error[0] - 2.0f * this->output_.error[1] + this->output_.error[2]);
        this->output_.Dout = this->factors_.Kd * this->output_.Dbuf[0];

        this->output_.Out += this->output_.Pout + this->output_.Iout + this->output_.Dout;

        return this->output_.Out;
    }

    fp32 SimplePidController::PidCalc(fp32 Set, fp32 Ref) {
        switch (this->pidmode_) {
            case PID_POSITION: {
                PidCalcPosition(Set, Ref);
                break;
            }
            case PID_DELTA: {
                PidCalcDelta(Set, Ref);
                break;
            }
            default: {
                return 0;
            }
        }
    }

    fp32 SegmentPidController::PidSegmentCalc(fp32 Set, fp32 Ref) {
        int index = 0;
        for (; index < this->Num_segments_; index++) {
            if (INRANGE((Set - Ref), this->limits_[index].DownSegment, this->limits_[index].UpSegment)) {
                SimplePidController::PidSwitch(this->factors_[index]);//切换pid参数而不改变输入输出量、
                printf("seg%d: p:%f,i:%f,d:%f\r\n", index, this->factors_[index].Kp, this->factors_[index].Ki,
                       this->factors_[index].Kd);
                return SimplePidController::PidCalc(Set, Ref);
            }
        }
        //如果不在范围内就使用默认参数
        {
            SimplePidController::PidInit(default_factors_);//切换pid参数而不改变输入输出量
            SimplePidController::PidCalc(Set, Ref);
            printf("seg%d: p:%f,i:%f,d:%f\r\n", index, this->factors_[index].Kp, this->factors_[index].Ki,
                   this->factors_[index].Kd);
            return SimplePidController::PidCalc(Set, Ref);
        };
    }

    void SegmentPidController::PidInit(std::vector<Basefactors_t> &bfs, std::vector<Segment_t> &seg_) {
        if (bfs.size() != this->NumSegments_ && seg_.size() != this->NumSegments_) {
            //错误处理函数
            printf("error segment");
            return;
        } else {
            this->factors_ = bfs;
            this->limits_ = seg_;
            for (uint8_t i = 0; i < this->NumSegments_; i++) {
                printf("seg%d: p:%f,i:%f,d:%f", i, this->factors_[i].Kp, this->factors_[i].Ki, this->factors_[i].Kd);
                printf("seg%d: Lup:%f,Ldown:%f", i, this->limits_[i].UpSegment, this->limits_[i].DownSegment);
                printf("\r\n");
            }
        }

    }
}




