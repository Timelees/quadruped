/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/
#ifndef GAITRECEIVER_H
#define GAITRECEIVER_H


#include <ocs2_core/Types.h>
#include <ocs2_legged_robot/gait/GaitSchedule.h>
#include <ocs2_legged_robot/gait/ModeSequenceTemplate.h>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>

#include <mutex>
#include <ocs2_msgs/msg/mode_schedule.hpp>

#include "rclcpp/rclcpp.hpp"

namespace ocs2::legged_robot
{
    class GaitReceiver final : public SolverSynchronizedModule
    {
    public:
        GaitReceiver(const rclcpp::Node::SharedPtr& node,
                     std::shared_ptr<GaitSchedule> gaitSchedulePtr,
                     const std::string& robotName);

        void preSolverRun(scalar_t initTime, scalar_t finalTime,
                          const vector_t& currentState,
                          const ReferenceManagerInterface& referenceManager) override;

        void postSolverRun(const PrimalSolution& primalSolution) override
        {
        }

    private:
        void mpcModeSequenceCallback(
            const ocs2_msgs::msg::ModeSchedule::ConstSharedPtr& msg);

        std::shared_ptr<GaitSchedule> gaitSchedulePtr_;

        rclcpp::Subscription<ocs2_msgs::msg::ModeSchedule>::SharedPtr
        mpcModeSequenceSubscriber_;

        std::mutex receivedGaitMutex_;
        std::atomic_bool gaitUpdated_;
        ModeSequenceTemplate receivedGait_;
    };
}
#endif // GAITRECEIVER_H
