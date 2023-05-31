//
// Created by 22627 on 2023/5/30.
//
#include "Inc/FuzzyController.h"

namespace TraditionalController {

// 三角形隶属度函数
    static fp32 triangularMembership(float x, float a, float b, float c) {
        if (x <= a || x >= c) {
            return 0.0;
        } else if (x >= b && x <= c) {
            return (c - x) / (c - b);
        } else if (x >= a && x <= b) {
            return (x - a) / (b - a);
        } else {
            return 0.0;
        }
    }

// 梯形隶属度函数
    static fp32 trapezoidalMembership(float x, float a, float b, float c, float d) {
        if (x <= a || x >= d) {
            return 0.0;
        } else if (x >= b && x <= c) {
            return 1.0;
        } else if (x >= a && x < b) {
            return (x - a) / (b - a);
        } else if (x > c && x <= d) {
            return (d - x) / (d - c);
        } else {
            return 0.0;
        }
    }

// 高斯隶属度函数
    static fp32 gaussianMembership(float x, float mean, float sigma) {
        return std::exp(-0.5 * std::pow((x - mean) / sigma, 2));
    }

    void FuzzyPidController::PidInit(BaseFactors_t &_bfs, FuzzyFactorRange_t &_fuzzy_factor_range) {
        this->factors_ = _bfs;
        this->fuzzy_factor_range_ = _fuzzy_factor_range;
    }


    void FuzzyPidController::FuzzyPIDController(fp32 _e, fp32 _er) {

        LIMITBAND(_e, this->fuzzy_factor_range_.e_range_.up_segment, this->fuzzy_factor_range_.e_range_.down_segment);
        LIMITBAND(_er, this->fuzzy_factor_range_.er_range_.up_segment,
                  this->fuzzy_factor_range_.er_range_.down_segment);

        this->mapped_e = LinearMap(_e, this->fuzzy_factor_range_.e_range_.down_segment,
                                   this->fuzzy_factor_range_.e_range_.up_segment, NB, PB);
        this->mapped_er = LinearMap(_er, this->fuzzy_factor_range_.er_range_.down_segment,
                                    this->fuzzy_factor_range_.er_range_.up_segment, NB, PB);

        /*******计算误差的隶属度*******/
#if MEMBERSHIPFUNCTION == triangular
        //计算误差的隶属度
        for (uint8_t index = 0; index <= 5; index++) {
            if (INRANGE(this->mapped_e, Domain[index], Domain[index + 1]))//判断误差所在区间
            {
                if (index != 0 && index != 5) {
                    this->e_memberShip_[0] = triangularMembership(this->mapped_e, Domain[index - 1], Domain[index],
                                                                  Domain[index + 1]);
                    this->e_memberShip_[1] = triangularMembership(this->mapped_e, Domain[index], Domain[index + 1],
                                                                  Domain[index + 2]);
                    e_index_[0] = index;
                    e_index_[1] = index + 1;
                    break;
                } else if (index == 0) {
                    this->e_memberShip_[0] = triangularMembership(this->mapped_e, -4, -3, -2);
                    this->e_memberShip_[1] = triangularMembership(this->mapped_e, Domain[index], Domain[index + 1],
                                                                  Domain[index + 2]);
                    e_index_[0] = 0;
                    e_index_[1] = 1;
                    break;
                } else if (index == 5) {
                    this->e_memberShip_[0] = triangularMembership(this->mapped_e, Domain[index - 1], Domain[index],
                                                                  Domain[index + 1]);
                    this->e_memberShip_[1] = triangularMembership(this->mapped_e, 2, 3, 4);
                    e_index_[0] = 5;
                    e_index_[1] = 6;
                    break;
                }
            }
        }
        //计算误差变化率的隶属度
        for (uint8_t index = 0; index <= 5; index++) {
            if (INRANGE(this->mapped_er, Domain[index], Domain[index + 1]))//判断误差所在区间
            {
                if (index != 0 && index != 5) {
                    this->er_memberShip_[0] = triangularMembership(this->mapped_er, Domain[index - 1], Domain[index],
                                                                   Domain[index + 1]);
                    this->er_memberShip_[1] = triangularMembership(this->mapped_er, Domain[index], Domain[index + 1],
                                                                   Domain[index + 2]);
                    er_index_[0] = index;
                    er_index_[1] = index + 1;
                    break;
                } else if (index == 0) {
                    this->er_memberShip_[0] = triangularMembership(this->mapped_er, -4, -3, -2);
                    this->er_memberShip_[1] = triangularMembership(this->mapped_er, Domain[index], Domain[index + 1],
                                                                   Domain[index + 2]);
                    er_index_[0] = 0;
                    er_index_[1] = 1;
                    break;
                } else if (index == 5) {
                    this->er_memberShip_[0] = triangularMembership(this->mapped_er, Domain[index - 1], Domain[index],
                                                                   Domain[index + 1]);
                    this->er_memberShip_[1] = triangularMembership(this->mapped_er, 2, 3, 4);
                    er_index_[0] = 5;
                    er_index_[1] = 6;
                    break;
                }
            }
        }
        printf("this error:%f\r\n",this->e_[0]);
        printf("the error1 is %f,the error2  is %f\r\n", Domain[e_index_[0]], Domain[e_index_[1]]);
        printf("the error1 membership is %f,the error2 membership is %f\r\n", this->e_memberShip_[0],
               this->e_memberShip_[1]);
        printf("this Rerror:%f\r\n",this->er_);
        printf("the Rerror1 is %f,the Rerror2  is %f\r\n", Domain[er_index_[0]], Domain[er_index_[1]]);
        printf("the Rerror1 membership is %f,the Rerror2 membership is %f\r\n", this->er_memberShip_[0],
               this->er_memberShip_[1]);
        /*******模糊规则处理*******/
        for (size_t i = 0; i < e_index_.size(); i++) {
            if (e_index_[i] == -1)
                continue;
            for (size_t j = 0; j < er_index_.size(); j++) {
                if (er_index_[j] == -1)
                    continue;
                this->delat_kp_ += (KpRulesTable[this->e_index_[i]][this->er_index_[j]] * this->e_memberShip_[i] *
                                    this->er_memberShip_[j]);
                this->delat_ki_ += (KiRulesTable[this->e_index_[i]][this->er_index_[j]] * this->e_memberShip_[i] *
                                    this->er_memberShip_[j]);
                this->delat_kd_ += (KdRulesTable[this->e_index_[i]][this->er_index_[j]] * this->e_memberShip_[i] *
                                    this->er_memberShip_[j]);
            }
        }
        /*******清晰化处理*******/
        //由于采取的规则是用于pid的增量设计 如果需要直接得到模糊化的pid参数 需修改pid的规则
#elif MEMBERSHIPFUNCTION == trapezoidal

#elif MEMBERSHIPFUNCTION==gaussian

#endif
    }

    fp32 FuzzyPidController::FuzzyPIDCalc(fp32 _set, fp32 _ref) {
        this->e_[1] = this->e_[0];
        this->er_ = this->output_.error[0] - this->output_.error[1];

        this->e_[0] = _set - _ref;

        this->FuzzyPIDController(this->e_[0], this->er_);

        this->factors_.kp += this->delat_kp_;
        this->factors_.ki += this->delat_ki_;
        this->factors_.kd += this->delat_kd_;

        printf("kp:%f,ki:%f,kd:%f\r\n", this->factors_.kp, this->factors_.ki, this->factors_.kd);
        printf("pout:%f,iout:%f,dout:%f\r\n", this->output_.p_out, this->output_.i_out, this->output_.d_out);

        LIMITBAND(this->factors_.kp, this->fuzzy_factor_range_.kp_range_.up_segment,
                  this->fuzzy_factor_range_.kp_range_.down_segment);
        LIMITBAND(this->factors_.ki, this->fuzzy_factor_range_.ki_range_.up_segment,
                  this->fuzzy_factor_range_.ki_range_.down_segment);
        LIMITBAND(this->factors_.kd, this->fuzzy_factor_range_.kd_range_.up_segment,
                  this->fuzzy_factor_range_.kd_range_.down_segment);

        return this->PidCalc(_set, _ref);
    }

}