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

    void FuzzyPidController::FuzzyPIDController(fp32 _e, fp32 _er) {
        this->mapped_e = LinearMap(_e, InEMin, InEMax, NB, PB);
        this->mapped_er = LinearMap(_e, InERMin, InERMax, NB, PB);

        /*******计算误差的隶属度*******/
#if MEMBERSHIPFUNCTION == triangular
        //计算误差的隶属度
        for (uint8_t index = 0; index < 5; index++) {
            if (INRANGE(_e, Domain[index], Domain[index + 1]))//判断误差所在区间
            {
                if (index != 0 && index != 5) {
                    triangularMembership(_e, Domain[index], Domain[index + 1], Domain[index + 2]);

                    e_index[0] = index;
                    e_index[1] = index + 1;
                    break;
                } else {
                    this->e_memberShip_.push_back(
                            triangularMembership(_e, -7 * (-1 ^ index), Domain[index], Domain[index + 1]));
                    this->e_memberShip_.push_back(
                            triangularMembership(_e, Domain[index], Domain[index + 1], Domain[index + 2]));

                    e_index[0] = (uint8_t) (6 * index) / 5;
                    e_index[1] = -1;
                    break;
                }
            }
        }
        //计算误差变化率的隶属度
        for (uint8_t index = 0; index < 5; index++) {
            if (INRANGE(_er, Domain[index], Domain[index + 1]))//判断误差所在区间
            {
                if (index != 0 && index != 5) {
                    triangularMembership(_er, Domain[index], Domain[index + 1], Domain[index + 2]);
                    break;
                } else {
                    this->er_memberShip_.push_back(
                            triangularMembership(_er, -7 * (-1 ^ index), Domain[index], Domain[index + 1]));
                    this->er_memberShip_.push_back(
                            triangularMembership(_er, Domain[index], Domain[index + 1], Domain[index + 2]));
                    break;
                }
            }
        }
#elif MEMBERSHIPFUNCTION == trapezoidal

#elif MEMBERSHIPFUNCTION==gaussian

#endif
        /*******模糊规则处理*******/
        /*******清晰化处理*******/
        this->delat_kp_ =
                (KpRulesTable[this->e_index[0]][this->er_index[0]] * this->e_memberShip_[0] * this->er_memberShip_[0]) + \
                (KpRulesTable[this->e_index[0]][this->er_index[1]] * this->e_memberShip_[0] * this->er_memberShip_[1]) + \
                (KpRulesTable[this->e_index[1]][this->er_index[0]] * this->e_memberShip_[1] * this->er_memberShip_[0]) + \
                (KpRulesTable[this->e_index[1]][this->er_index[1]] * this->e_memberShip_[1] * this->er_memberShip_[1]);

        this->delat_ki_ =
                (KiRulesTable[this->e_index[0]][this->er_index[0]] * this->e_memberShip_[0] * this->er_memberShip_[0]) + \
                (KiRulesTable[this->e_index[0]][this->er_index[1]] * this->e_memberShip_[0] * this->er_memberShip_[1]) + \
                (KiRulesTable[this->e_index[1]][this->er_index[0]] * this->e_memberShip_[1] * this->er_memberShip_[0]) + \
                (KiRulesTable[this->e_index[1]][this->er_index[1]] * this->e_memberShip_[1] * this->er_memberShip_[1]);

        this->delat_kd_ =
                (KdRulesTable[this->e_index[0]][this->er_index[0]] * this->e_memberShip_[0] * this->er_memberShip_[0]) + \
                (KdRulesTable[this->e_index[0]][this->er_index[1]] * this->e_memberShip_[0] * this->er_memberShip_[1]) + \
                (KdRulesTable[this->e_index[1]][this->er_index[0]] * this->e_memberShip_[1] * this->er_memberShip_[0]) + \
                (KdRulesTable[this->e_index[1]][this->er_index[1]] * this->e_memberShip_[1] * this->er_memberShip_[1]);
    }

    void FuzzyPidController::FuzzyPIDCalc(fp32 _set, fp32 _ref) {
        this->e_[1] = this->e_[0];
        this->er_ = this->output_.error[0] - this->output_.error[1];

        this->e_[0] = _set - _ref;

        this->FuzzyPIDController(this->e_[0], this->er_);

        this->factors_.kp += this->delat_kp_;
        this->factors_.ki += this->delat_ki_;
        this->factors_.kd += this->delat_kd_;


        this->PidCalc(_set, _ref);

    }

}