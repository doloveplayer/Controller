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

    fp32 FuzzyPidController::FuzzyPIDCalc(fp32 _set, fp32 _ref) {
        this->e_[1] = this->e_[0];
        this->er_ = this->output_.error[0] - this->output_.error[1];

        this->e_[0] = _set - _ref;

        this->FuzzyPIDController(this->e_[0], this->er_);

        LIMITBAND(this->factors_.kp, this->fuzzy_factor_range_.kp_range_.up_segment,
                  this->fuzzy_factor_range_.kp_range_.down_segment);
        LIMITBAND(this->factors_.ki, this->fuzzy_factor_range_.ki_range_.up_segment,
                  this->fuzzy_factor_range_.ki_range_.down_segment);
        LIMITBAND(this->factors_.kd, this->fuzzy_factor_range_.kd_range_.up_segment,
                  this->fuzzy_factor_range_.kd_range_.down_segment);

        return this->PidCalc(_set, _ref);
    }

    void FuzzyPidController::FuzzyPIDController(fp32 _e, fp32 _er) {
        /*******对输入变量限幅*******/
        LIMITBAND(_e, this->fuzzy_factor_range_.e_range_.up_segment, this->fuzzy_factor_range_.e_range_.down_segment);
        LIMITBAND(_er, this->fuzzy_factor_range_.er_range_.up_segment,
                  this->fuzzy_factor_range_.er_range_.down_segment);
        /*******映射到论域*******/
        this->mapped_e = LinearMap(_e, this->fuzzy_factor_range_.e_range_.down_segment,
                                   this->fuzzy_factor_range_.e_range_.up_segment, NB, PB);
        this->mapped_er = LinearMap(_er, this->fuzzy_factor_range_.er_range_.down_segment,
                                    this->fuzzy_factor_range_.er_range_.up_segment, NB, PB);
        /*******计算隶属度*******/
        this->CalculateMembership();
        /*******模糊规则处理*******/
        this->CalculateFuzzyRules();
        /*******清晰化处理*******/
        this->factors_.kp += this->delat_kp_ * KpRight;
        this->factors_.ki += this->delat_ki_ * KiRight;
        this->factors_.kd += this->delat_kd_ * KdRight;
        printf("kp:%f,ki:%f,kd:%f\r\n", this->factors_.kp, this->factors_.ki, this->factors_.kd);
        printf("pout:%f,iout:%f,dout:%f\r\n", this->output_.p_out, this->output_.i_out, this->output_.d_out);
        //由于采取的规则是用于pid的增量设计 如果需要直接得到模糊化的pid参数 需修改pid的规则
    }

    void FuzzyPidController::CalculateMembership() {
        uint8_t domain_size = (uint8_t) (sizeof(Domain) / 4);
        /*******计算隶属度*******/
#if MEMBERSHIPFUNCTION == triangular
        //计算误差的隶属度
        if (this->mapped_e > Domain[0] && this->mapped_e < Domain[domain_size - 1]) {//计算包含在论域内的隶属度
            for (uint8_t j = 0; j < domain_size; j++) {
                this->e_membership_index_.insert(std::make_pair(j, triangularMembership(this->mapped_e,
                                                                                        Domain[j] - MEMBERSHIP_STEP,
                                                                                        Domain[j],
                                                                                        Domain[j] +
                                                                                        MEMBERSHIP_STEP)));
            }
        } else {//计算不包含在论域内的隶属度
            if (this->mapped_e <= Domain[0]) {
                this->e_membership_index_.insert(std::make_pair(0, 1));
            } else if (this->mapped_e >= Domain[domain_size - 1]) {
                this->e_membership_index_.insert(std::make_pair(domain_size - 1, 1));
            }
        }

        //计算误差变化率的隶属度
        if (this->mapped_er > Domain[0] && this->mapped_er < Domain[domain_size - 1]) {//计算包含在论域内的隶属度
            for (uint8_t j = 0; j < domain_size; j++) {
                this->er_membership_index_.insert(std::make_pair(j, triangularMembership(this->mapped_er,
                                                                                         Domain[j] -
                                                                                         MEMBERSHIP_STEP,
                                                                                         Domain[j],
                                                                                         Domain[j] +
                                                                                         MEMBERSHIP_STEP)));
                if (this->er_membership_index_[j] == 0) {//如果隶属度为零 说明已经不在能包括他的论域了
                    this->er_membership_index_.erase(j);
                    break;
                }
            }
        } else {//计算不包含在论域内的隶属度
            if (this->mapped_er <= Domain[0]) {
                this->er_membership_index_.insert(std::make_pair(0, 1));
            } else if (this->mapped_er >= Domain[domain_size - 1]) {
                this->er_membership_index_.insert(std::make_pair(domain_size - 1, 1));
            }

        }
#elif MEMBERSHIPFUNCTION == trapezoidal

#elif MEMBERSHIPFUNCTION==gaussian

#endif
    }

    void FuzzyPidController::CalculateFuzzyRules() {
        std::map<int8_t, fp32>::iterator e_iter;
        std::map<int8_t, fp32>::iterator er_iter;
        for (e_iter = e_membership_index_.begin(); e_iter != e_membership_index_.end(); e_iter++) {
            for (er_iter = er_membership_index_.begin(); er_iter != er_membership_index_.end(); er_iter++) {
                this->delat_kp_ += (KpRulesTable[e_iter->first][er_iter->first] * e_iter->second * er_iter->second);
                this->delat_ki_ += (KiRulesTable[e_iter->first][er_iter->first] * e_iter->second * er_iter->second);
                this->delat_kd_ += (KdRulesTable[e_iter->first][er_iter->first] * e_iter->second * er_iter->second);
            }
        }
    }
}
