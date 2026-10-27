#pragma once

#include <atomic>
#include <deque>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

namespace sitl_mission_node {

// MQTT 3.1.1，QoS 0。订阅任务，发布状态。断线在线程里重连，不阻塞 ROS executor。
class MqttTaskClient {
public:
  using Handler = std::function<void(const std::string &)>;
  using Logger = std::function<void(const std::string &)>;

  MqttTaskClient(
    std::string host,
    int port,
    std::string command_topic,
    std::string status_topic,
    Handler handler,
    Logger logger);
  ~MqttTaskClient();

  void start();
  void publish_status(const std::string & payload);

private:
  void run();
  bool session_once();
  void wake();

  std::string host_;
  int port_;
  std::string topic_;          // 订阅：任务命令。
  std::string status_topic_;   // 发布：和 /patrol_status 同一段文本。
  Handler handler_;            // 只许入队，不许在 MQTT 线程里改阶段。
  Logger logger_;
  std::atomic<bool> stop_{false};
  std::thread thread_;
  std::mutex outgoing_mutex_;
  std::deque<std::string> outgoing_;  // 最多 16 条，满了丢最旧的。
  int wake_fds_[2]{-1, -1};           // pipe。写一端把读循环从 poll 里叫醒。
};

}  // namespace sitl_mission_node
