#pragma once
#include <rclcpp/rclcpp.hpp>

#include "topstar_api/msg/request.hpp"
#include "topstar_api/msg/response.hpp"

namespace libstatistics_collector::topic_statistics_collector {
/**
 * Just a patch for ros2 foxy, more information can be tracked
 * through the following methods https://github.com/ros2/ros2/issues/1324.
 */
template <>
struct TimeStamp<topstar_api::msg::Response> {
  static std::pair<bool, int64_t> value(
      const topstar_api::msg::Response& /*m*/) {
    return std::make_pair(true, 0);
  }
};
}  // namespace libstatistics_collector::topic_statistics_collector
