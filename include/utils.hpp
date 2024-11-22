#pragma once

#include "common.hpp"

#include "json.hpp"

#include <Eigen/Dense>

namespace utils
{

using json = nlohmann::json;

/**
 * @brief      Parses the configuration from a Json file
 *
 * @param      config_in  The configuration in
 *
 * @return     The Config struct generated
 */
Config parse(json const& config_in);

Eigen::Vector2d closestPointInLine(const Eigen::Vector2d& line_start, const Eigen::Vector2d& line_end,
                                   const Eigen::Vector2d& point);

}  // namespace utils