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
        fp32 temp_out = 0;
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
        //积分分离
        if (ABS(this->output_.error[0]) > this->factors_.band_i) {
            this->output_.i_out += 0;
        } else {
            //实际使用如果不需要这些复杂的方法直接注释掉以提高运行效率
            if(advanced_factors_.variable_integral_.flag==1 && advanced_factors_.trapezium_integral_.flag==0)//变速积分
            {
                this->output_.i_out += this->factors_.ki * this->output_.error[0]\
                *advanced_factors_.VariableIntergralCoefficientCalc(advanced_factors_.variable_integral_.max_intergral,advanced_factors_.variable_integral_.min_intergral,this->output_.error[0] );
            }
            else if(advanced_factors_.trapezium_integral_.flag==1 && advanced_factors_.variable_integral_.flag==0)//梯形积分
            {
                this->output_.i_out += this->factors_.ki * (this->output_.error[0]+this->output_.error[1])/2;
            }
            else if(advanced_factors_.trapezium_integral_.flag==1 && advanced_factors_.variable_integral_.flag==1)//变速且梯形
            {
                this->output_.i_out += this->factors_.ki * ((this->output_.error[0]+this->output_.error[1])/2)\
                *advanced_factors_.VariableIntergralCoefficientCalc(advanced_factors_.variable_integral_.max_intergral,advanced_factors_.variable_integral_.min_intergral,this->output_.error[0] );
            }else
            {
                this->output_.i_out += this->factors_.ki * this->output_.error[0];
            }
        }
        //积分限幅
        LIMITMAX(this->output_.i_out, this->factors_.max_i_out);
        //计算d
        this->output_.d_buf[2] = this->output_.d_buf[1];
        this->output_.d_buf[1] = this->output_.d_buf[0];
        this->output_.d_buf[0] = (this->output_.error[0] - this->output_.error[1]);
        this->output_.d_out = this->factors_.kd * this->output_.d_buf[0];

        this->output_.out = this->output_.p_out + this->output_.i_out + this->output_.d_out;
        if(advanced_factors_.forward_feed_.flag == 1)//前馈
        {
            temp_out=advanced_factors_.SegmentForwardFeed(this->output_.error[0]);
            printf("feed outr %f",temp_out);
            this->output_.out += temp_out;
        }
        //输出限幅
        LIMITMAX(this->output_.out, this->factors_.max_out);

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
            return 0;
        }
        //计算p
        this->output_.p_out = this->factors_.kp * (this->output_.error[0] - this->output_.error[1]);
        //计算i
        //积分分离
        if (ABS(this->output_.error[0]) > this->factors_.band_i) {
            this->output_.i_out += 0;
        } else {
            this->output_.i_out += this->factors_.ki * this->output_.error[0];
        }
        //积分限幅
        LIMITMAX(this->output_.i_out, this->factors_.max_i_out);
        //计算d
        this->output_.d_buf[2] = this->output_.d_buf[1];
        this->output_.d_buf[1] = this->output_.d_buf[0];
        this->output_.d_buf[0] = (this->output_.error[0] - 2.0f * this->output_.error[1] + this->output_.error[2]);
        this->output_.d_out = this->factors_.kd * this->output_.d_buf[0];

        this->output_.out = this->output_.p_out + this->output_.i_out + this->output_.d_out;
        //输出限幅
        LIMITMAX(this->output_.out, this->factors_.max_out);

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

    void SimplePidController::PidClear() {
        {
            uint8_t *ptr = nullptr;
            ptr = (uint8_t *) &output_;
            for (int index = 0; index < sizeof(output_); ptr++) {
                if (ptr != nullptr) {
                    *ptr = 0;
                } else {
                    ptr = (uint8_t *) &input_;
                    break;
                }
            }
            for (int index = 0; index < sizeof(input_); ptr++) {
                if (ptr != nullptr) {
                    *ptr = 0;
                } else {
                    ptr = (uint8_t *) &factors_;
                    break;
                }
            }
            for (int index = 0; index < sizeof(factors_); ptr++) {
                if (ptr != nullptr) {
                    *ptr = 0;
                } else {
                    break;
                }
            }
        };
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

    fp32 AdvancedFactors::ForwardFeed(fp32 _in)
    {
        float out=0.0;
        if(this->forward_feed_.time == 0)
        {
            return _in;
        }
        out = (_in-this->forward_feed_.last_in)/this->forward_feed_.time + _in;
        this->forward_feed_.last_in = _in;
        return out;
    }

    void AdvancedFactors::SegmentForwardFeedInit(std::vector<Segment_t> &_ref_seg,std::vector<fp32> &_out)
    {
        this->forward_feed_.ref_segment.resize(_ref_seg.size());
        this->forward_feed_.out.resize(_out.size());

        this->forward_feed_.last_in =0;
        this->forward_feed_.flag =1;
        //获取参数
        this->forward_feed_.ref_segment = _ref_seg;
        this->forward_feed_.out = _out;
        //记录分段数量
        if(_ref_seg.size()!=_out.size())
        {
            //错误处理函数
            return;
        }
        this->forward_feed_.segment_size = _out.size();
        printf("out num : %d ",this->forward_feed_.segment_size);
    }

    fp32 AdvancedFactors::SegmentForwardFeed(fp32 _err)
    {
        int index = 0;
        for (; index < this->forward_feed_.segment_size; index++) {
            if (INRANGE(_err, this->forward_feed_.ref_segment[index].down_segment, this->forward_feed_.ref_segment[index].up_segment)) {
                return this->forward_feed_.out[index];
            }
        }
        return 0;
    }

    void AdvancedFactors::VariableIntergralInit(fp32 _max_interval, fp32 _min_interval)
    {
        if(_max_interval<_min_interval)
        {
            //错误处理函数
            return;
        }
        this->variable_integral_.flag =1;
        this->variable_integral_.max_intergral =_max_interval;
        this->variable_integral_.min_intergral = _min_interval;
    }

    fp32 AdvancedFactors::VariableIntergralCoefficientCalc(fp32 &_max_interval, fp32 &_min_interval, fp32 &_error)
    {
        if (ABS(_error) <= _min_interval)
        {
            return 1.0;
        }
        else if (ABS(_error) >_max_interval)
        {
            return 0;
        }
        else
        {
            return ((_max_interval - ABS(_error)) / (_max_interval - _min_interval));
        }
    }


}




