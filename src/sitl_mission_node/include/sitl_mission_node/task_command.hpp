#pragma once

#include <algorithm>
#include <cctype>
#include <string>

namespace sitl_mission_node {

enum class TaskCommand {
  Start,
  Cancel,
  Ignore
};

inline std::string trim_lower_copy(std::string text)
{
  auto not_space = [](unsigned char c) { return std::isspace(c) == 0; };
  const auto begin = std::find_if(text.begin(), text.end(), not_space);
  const auto end = std::find_if(text.rbegin(), text.rend(), not_space).base();
  if (begin >= end) {
    return {};
  }
  std::string out(begin, end);
  for (char & c : out) {
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  }
  return out;
}

// MQTT 与 /patrol_task 共用同一份文本：start / cancel。
inline TaskCommand parse_task_command(const std::string & payload)
{
  const std::string text = trim_lower_copy(payload);
  if (text == "start") {
    return TaskCommand::Start;
  }
  if (text == "cancel") {
    return TaskCommand::Cancel;
  }
  return TaskCommand::Ignore;
}

}  // namespace sitl_mission_node
