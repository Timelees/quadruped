#include "ocs2_quadruped_controller/perceptive/common/FootPlacementCBFLogger.h"

#include <sstream>

namespace ocs2::legged_robot
{
    FootPlacementCBFLogger::FootPlacementCBFLogger()
    {
        logger_ = std::make_unique<quadruped::logs::LogSave>();
        logger_->setDefaultFilePath("logs/data/wbc_foot_placement_cbf.csv");
    }

    void FootPlacementCBFLogger::setEnabled(bool enabled)
    {
        enabled_ = enabled;
        if (logger_)
        {
            logger_->setEnabled(enabled);
        }
    }

    void FootPlacementCBFLogger::logSample(scalar_t time,
                                           size_t leg,
                                           scalar_t phase,
                                           scalar_t shrinkMargin,
                                           scalar_t hMin,
                                           scalar_t hDotMin,
                                           scalar_t hEMin,
                                           scalar_t ecbfResidualMin)
    {
        if (!logger_ || !logger_->isEnabled())
        {
            return;
        }

        if (!headerWritten_)
        {
            logger_->open();
            logger_->appendLine("time,leg,phase,shrink_margin,h_min,h_dot_min,h_e_min,ecbf_residual_min");
            headerWritten_ = true;
        }

        std::ostringstream oss;
        oss << time << ',' << leg << ',' << phase << ',' << shrinkMargin << ','
            << hMin << ',' << hDotMin << ',' << hEMin << ',' << ecbfResidualMin;
        logger_->appendLine(oss.str());
    }
} // namespace ocs2::legged_robot
