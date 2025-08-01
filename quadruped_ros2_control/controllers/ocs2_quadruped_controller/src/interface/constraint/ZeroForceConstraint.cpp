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

#include "ocs2_quadruped_controller/interface/constraint/ZeroForceConstraint.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>

/* 确保摆动相足端的接触力为0 */
namespace ocs2::legged_robot {
    ZeroForceConstraint::ZeroForceConstraint(const SwitchedModelReferenceManager &referenceManager,
                                             size_t contactPointIndex,
                                             CentroidalModelInfo info)
        : StateInputConstraint(ConstraintOrder::Linear),
          referenceManagerPtr_(&referenceManager),
          contactPointIndex_(contactPointIndex),
          info_(std::move(info)) {
    }


    bool ZeroForceConstraint::isActive(scalar_t time) const {
        return !referenceManagerPtr_->getContactFlags(time)[contactPointIndex_];
    }


    vector_t ZeroForceConstraint::getValue(scalar_t time, const vector_t &state, const vector_t &input,
                                           const PreComputation &preComp) const {
        return centroidal_model::getContactForces(input, contactPointIndex_, info_);
    }


    VectorFunctionLinearApproximation ZeroForceConstraint::getLinearApproximation(
        scalar_t time, const vector_t &state, const vector_t &input,
        const PreComputation &preComp) const {
        VectorFunctionLinearApproximation approx;
        approx.f = getValue(time, state, input, preComp);
        approx.dfdx = matrix_t::Zero(3, state.size());
        approx.dfdu = matrix_t::Zero(3, input.size());
        approx.dfdu.middleCols<3>(3 * contactPointIndex_).diagonal() = vector_t::Ones(3);
        return approx;
    }
} // namespace ocs2::legged_robot
