//
// Diagnostic logger for WBC foot placement CBF metrics.
//

#pragma once

#include <memory>
#include <string>

#include "log_save.h"

namespace ocs2::legged_robot
{
    class FootPlacementCBFLogger
    {
    public:
        FootPlacementCBFLogger();

        void setEnabled(bool enabled);

        bool isEnabled() const { return enabled_; }

        void logSample(scalar_t time,
                       size_t leg,
                       scalar_t phase,
                       scalar_t shrinkMargin,
                       scalar_t hMin,
                       scalar_t hDotMin,
                       scalar_t hEMin,
                       scalar_t ecbfResidualMin);

    private:
        std::unique_ptr<quadruped::logs::LogSave> logger_;
        bool headerWritten_{false};
        bool enabled_{false};
    };
} // namespace ocs2::legged_robot
