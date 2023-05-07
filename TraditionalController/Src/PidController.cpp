//
// Created by 22627 on 2023/5/1.
//

#include "Inc/PidController.h"

namespace PidController {

    void SimplePidController::PidInit(Basefactors_t &bfs) {
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
            return 0;
        }
        //计算p
        this->output_.Pout = this->factors_.Kp * this->output_.error[0];
        //计算i
        this->output_.Iout += this->factors_.Ki * this->output_.error[0];
        LimitMax(this->output_.Iout, this->factors_.MaxIout);
        //计算d
        this->output_.Dbuf[2] = this->output_.Dbuf[1];
        this->output_.Dbuf[1] = this->output_.Dbuf[0];
        this->output_.Dbuf[0] = (this->output_.error[0] - this->output_.error[1]);
        this->output_.Dout = this->factors_.Kd * this->output_.Dbuf[0];

        this->output_.Out = this->output_.Pout + this->output_.Iout + this->output_.Dout;
        LimitMax(this->output_.Out, this->factors_.MaxOut);

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
            return 0;
        }
        //计算p
        this->output_.Pout = this->factors_.Kp * (this->output_.error[0] - this->output_.error[1]);
        //计算i
        this->output_.Iout += this->factors_.Ki * this->output_.error[0];
        LimitMax(this->output_.Iout, this->factors_.MaxIout);
        //计算d
        this->output_.Dbuf[2] = this->output_.Dbuf[1];
        this->output_.Dbuf[1] = this->output_.Dbuf[0];
        this->output_.Dbuf[0] = (this->output_.error[0] - 2.0f * this->output_.error[1] + this->output_.error[2]);
        this->output_.Dout = this->factors_.Kd * this->output_.Dbuf[0];

        this->output_.Out += this->output_.Pout + this->output_.Iout + this->output_.Dout;
        LimitMax(this->output_.Out, this->factors_.MaxOut);

        return this->output_.Out;
    }

    fp32 SimplePidController::PidCalc(fp32 Set, fp32 Ref)
    {
        switch (this->pidmode_) {

            case PID_POSITION:
                PidCalcPosition(Set, Ref);
            case PID_DELTA:
                PidCalcDelta(Set, Ref);
            default:
                return 0;
        }
        return 0;
    }
    fp32 SegmentPidController::PidSegmentCalc(fp32 Set, fp32 Ref) {
        int index = 0;
        for (; index < this->NumSegments_; index++) {
            if (!INRANGE(Set, this->segments_[index].DownSegment, this->segments_[index].UpSegment)) {
                SimplePidController::PidInit(this->factors_[index]);//切换pid参数而不改变输入输出量
                SimplePidController::PidCalc(Set, Ref);
            };
        }
        return 0;
    }
}




