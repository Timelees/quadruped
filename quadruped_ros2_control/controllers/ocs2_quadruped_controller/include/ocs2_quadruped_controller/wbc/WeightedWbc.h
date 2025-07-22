//
// Created by qiayuan on 22-12-23.
//
#pragma once

#include "WbcBase.h"

namespace ocs2::legged_robot
{
    class WeightedWbc final : public WbcBase
    {
    public:
        using WbcBase::WbcBase;

        vector_t update(const vector_t& stateDesired, const vector_t& inputDesired, const vector_t& rbdStateMeasured,
                        size_t mode,
                        scalar_t period) override;

        void loadTasksSetting(const std::string& taskFile, bool verbose) override;

    protected:
        Task formulateConstraints();

        Task formulateWeightedTasks(const vector_t& stateDesired, const vector_t& inputDesired,
                                    scalar_t period);

    private:
        
        scalar_t weightSwingLeg_;       // 摆动腿任务的权重，控制摆动腿运动的优先级
        scalar_t weightBaseAccel_;      // 基座加速度任务的权重，控制基座运动的优先级
        scalar_t weightContactForce_;   // 接触力任务的权重，控制足端力的优化优先级
    };
} // namespace legged
