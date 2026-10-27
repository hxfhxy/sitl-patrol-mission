#include "sitl_mission_node/mqtt_task_client.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

#include <gtest/gtest.h>

namespace {

bool take_packet(std::string & buffer, std::string & packet)
{
  if (buffer.size() < 2) {
    return false;
  }
  size_t index = 1;
  uint32_t multiplier = 1;
  uint32_t value = 0;
  uint8_t encoded = 0;
  do {
    if (index >= buffer.size() || multiplier > 128u * 128u * 128u) {
      return false;
    }
    encoded = static_cast<uint8_t>(buffer[index++]);
    value += static_cast<uint32_t>(encoded & 127u) * multiplier;
    multiplier *= 128u;
  } while ((encoded & 0x80u) != 0);
  if (buffer.size() < index + value) {
    return false;
  }
  packet.assign(buffer.begin(), buffer.begin() + static_cast<std::ptrdiff_t>(index + value));
  buffer.erase(0, index + value);
  return true;
}

std::string encode_publish(const std::string & topic, const std::string & payload)
{
  std::string body;
  body.push_back(static_cast<char>((topic.size() >> 8) & 0xff));
  body.push_back(static_cast<char>(topic.size() & 0xff));
  body += topic;
  body += payload;
  std::string packet;
  packet.push_back(static_cast<char>(0x30));
  size_t length = body.size();
  do {
    uint8_t encoded = static_cast<uint8_t>(length % 128);
    length /= 128;
    if (length > 0) {
      encoded = static_cast<uint8_t>(encoded | 0x80);
    }
    packet.push_back(static_cast<char>(encoded));
  } while (length > 0);
  packet += body;
  return packet;
}

bool send_all(int fd, const std::string & bytes)
{
  size_t sent = 0;
  while (sent < bytes.size()) {
    const ssize_t n = ::send(fd, bytes.data() + sent, bytes.size() - sent, MSG_NOSIGNAL);
    if (n < 0) {
      if (errno == EINTR) {
        continue;
      }
      return false;
    }
    sent += static_cast<size_t>(n);
  }
  return true;
}

bool recv_packet(int fd, std::string & buffer, std::string & packet)
{
  while (!take_packet(buffer, packet)) {
    char tmp[512];
    const ssize_t n = ::recv(fd, tmp, sizeof(tmp), 0);
    if (n <= 0) {
      return false;
    }
    buffer.append(tmp, static_cast<size_t>(n));
  }
  return true;
}

std::string publish_payload(const std::string & packet)
{
  if (packet.size() < 5 || (static_cast<uint8_t>(packet[0]) >> 4) != 3) {
    return {};
  }
  size_t offset = 1;
  uint8_t encoded = 0;
  do {
    encoded = static_cast<uint8_t>(packet[offset++]);
  } while ((encoded & 0x80u) != 0 && offset < packet.size());
  if (offset + 2 > packet.size()) {
    return {};
  }
  const uint16_t topic_len = static_cast<uint16_t>(
    (static_cast<uint8_t>(packet[offset]) << 8) | static_cast<uint8_t>(packet[offset + 1]));
  offset += 2 + topic_len;
  if (offset > packet.size()) {
    return {};
  }
  return packet.substr(offset);
}

class FakeBroker {
public:
  FakeBroker()
  {
    listen_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    int one = 1;
    ::setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = 0;
    if (::bind(listen_fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0 ||
      ::listen(listen_fd_, 1) != 0)
    {
      return;
    }
    socklen_t len = sizeof(addr);
    ::getsockname(listen_fd_, reinterpret_cast<sockaddr *>(&addr), &len);
    port_ = ntohs(addr.sin_port);
    thread_ = std::thread([this]() { serve(); });
  }

  ~FakeBroker()
  {
    if (listen_fd_ >= 0) {
      ::shutdown(listen_fd_, SHUT_RDWR);
      ::close(listen_fd_);
      listen_fd_ = -1;
    }
    if (thread_.joinable()) {
      thread_.join();
    }
  }

  int port() const { return port_; }
  std::string status_payload() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return status_payload_;
  }

private:
  void serve()
  {
    const int client = ::accept(listen_fd_, nullptr, nullptr);
    if (client < 0) {
      return;
    }
    const int nodelay = 1;
    ::setsockopt(client, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));

    std::string buffer;
    std::string packet;
    if (!recv_packet(client, buffer, packet)) {
      ::close(client);
      return;
    }
    const std::string connack{"\x20\x02\x00\x00", 4};
    if (!send_all(client, connack) || !recv_packet(client, buffer, packet)) {
      ::close(client);
      return;
    }
    const std::string suback{"\x90\x03\x00\x01\x00", 5};
    if (!send_all(client, suback)) {
      ::close(client);
      return;
    }

    while (status_payload().empty()) {
      if (!recv_packet(client, buffer, packet)) {
        ::close(client);
        return;
      }
      const std::string payload = publish_payload(packet);
      if (!payload.empty()) {
        std::lock_guard<std::mutex> lock(mutex_);
        status_payload_ = payload;
      }
    }

    const std::string command = encode_publish("sitl/patrol/task", "start");
    send_all(client, command);
    while (recv_packet(client, buffer, packet)) {
    }
    ::close(client);
  }

  int listen_fd_{-1};
  int port_{0};
  std::thread thread_;
  mutable std::mutex mutex_;
  std::string status_payload_;
};

bool wait_until(const std::function<bool()> & ready)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (std::chrono::steady_clock::now() < deadline) {
    if (ready()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  return false;
}

// 本地假 broker：状态能发出去，start 能从订阅回到 handler。
TEST(MqttRoundtrip, StatusGoesOutAndCommandComesBack)
{
  FakeBroker broker;
  ASSERT_GT(broker.port(), 0);

  std::mutex mutex;
  std::string command;
  std::atomic<bool> subscribed{false};
  sitl_mission_node::MqttTaskClient client(
    "127.0.0.1", broker.port(), "sitl/patrol/task", "sitl/patrol/status",
    [&](const std::string & payload) {
      std::lock_guard<std::mutex> lock(mutex);
      command = payload;
    },
    [&](const std::string & line) {
      if (line.find("已订阅") != std::string::npos) {
        subscribed.store(true);
      }
    });
  client.start();

  ASSERT_TRUE(wait_until([&]() { return subscribed.load(); }));
  client.publish_status("fault scan_disabled");
  ASSERT_TRUE(wait_until([&]() { return broker.status_payload() == "fault scan_disabled"; }));
  ASSERT_TRUE(wait_until([&]() {
    std::lock_guard<std::mutex> lock(mutex);
    return command == "start";
  }));
}

}  // namespace
