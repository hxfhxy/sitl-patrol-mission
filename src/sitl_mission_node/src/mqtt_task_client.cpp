#include "sitl_mission_node/mqtt_task_client.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstring>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <utility>

namespace sitl_mission_node {

namespace {

void append_u16(std::string & out, uint16_t value)
{
  out.push_back(static_cast<char>((value >> 8) & 0xff));
  out.push_back(static_cast<char>(value & 0xff));
}

void append_remaining_length(std::string & out, size_t length)
{
  do {
    uint8_t encoded = static_cast<uint8_t>(length % 128);
    length /= 128;
    if (length > 0) {
      encoded = static_cast<uint8_t>(encoded | 0x80);
    }
    out.push_back(static_cast<char>(encoded));
  } while (length > 0);
}

std::string encode_publish(const std::string & topic, const std::string & payload)
{
  std::string body;
  append_u16(body, static_cast<uint16_t>(topic.size()));
  body += topic;
  body += payload;
  std::string packet;
  packet.push_back(static_cast<char>(0x30));
  append_remaining_length(packet, body.size());
  packet += body;
  return packet;
}

bool send_all(int fd, const std::string & bytes, const std::atomic<bool> & stop)
{
  size_t sent = 0;
  while (sent < bytes.size()) {
    if (stop.load()) {
      return false;
    }
    const ssize_t n = ::send(fd, bytes.data() + sent, bytes.size() - sent, MSG_NOSIGNAL);
    if (n < 0) {
      if (errno == EINTR) {
        continue;
      }
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        pollfd pfd{};
        pfd.fd = fd;
        pfd.events = POLLOUT;
        if (::poll(&pfd, 1, 200) < 0 && errno != EINTR) {
          return false;
        }
        continue;
      }
      return false;
    }
    if (n == 0) {
      return false;
    }
    sent += static_cast<size_t>(n);
  }
  return true;
}

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

int connect_broker(
  const std::string & host,
  int port,
  const std::atomic<bool> & stop,
  const MqttTaskClient::Logger & logger)
{
  addrinfo hints{};
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;
  addrinfo * result = nullptr;
  const std::string port_text = std::to_string(port);
  if (getaddrinfo(host.c_str(), port_text.c_str(), &hints, &result) != 0) {
    if (logger) {
      logger("MQTT 解析地址失败: " + host);
    }
    return -1;
  }

  int fd = -1;
  for (addrinfo * it = result; it != nullptr && !stop.load(); it = it->ai_next) {
    fd = ::socket(it->ai_family, it->ai_socktype, it->ai_protocol);
    if (fd < 0) {
      continue;
    }
    const int flags = ::fcntl(fd, F_GETFL, 0);
    ::fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    const int rc = ::connect(fd, it->ai_addr, it->ai_addrlen);
    if (rc == 0) {
      break;
    }
    bool connected = false;
    if (errno == EINPROGRESS) {
      for (int attempt = 0; attempt < 10 && !stop.load(); ++attempt) {
        pollfd pfd{};
        pfd.fd = fd;
        pfd.events = POLLOUT;
        const int waited = ::poll(&pfd, 1, 200);
        if (waited < 0) {
          if (errno == EINTR) {
            continue;
          }
          break;
        }
        if (waited == 0) {
          continue;
        }
        int so_error = 0;
        socklen_t len = sizeof(so_error);
        if (::getsockopt(fd, SOL_SOCKET, SO_ERROR, &so_error, &len) == 0 && so_error == 0) {
          connected = true;
        }
        break;
      }
    }
    if (connected) {
      break;
    }
    ::close(fd);
    fd = -1;
  }
  freeaddrinfo(result);
  if (fd < 0) {
    if (logger) {
      logger("MQTT 连接失败: " + host + ":" + port_text);
    }
    return -1;
  }

  const int flags = ::fcntl(fd, F_GETFL, 0);
  ::fcntl(fd, F_SETFL, flags | O_NONBLOCK);
  const int nodelay = 1;
  ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));
  return fd;
}

class BrokerConn
{
public:
  BrokerConn(
    int fd,
    std::atomic<bool> & stop,
    int wake_fd,
    std::mutex & outgoing_mutex,
    std::deque<std::string> & outgoing,
    std::string status_topic)
  : fd_(fd),
    stop_(stop),
    wake_fd_(wake_fd),
    outgoing_mutex_(outgoing_mutex),
    outgoing_(outgoing),
    status_topic_(std::move(status_topic)),
    last_io_(std::chrono::steady_clock::now())
  {
  }

  ~BrokerConn()
  {
    if (fd_ >= 0) {
      ::close(fd_);
    }
  }

  bool flush_outgoing()
  {
    std::deque<std::string> local;
    {
      std::lock_guard<std::mutex> lock(outgoing_mutex_);
      local.swap(outgoing_);
    }
    for (size_t i = 0; i < local.size(); ++i) {
      if (!send_all(fd_, encode_publish(status_topic_, local[i]), stop_)) {
        std::lock_guard<std::mutex> lock(outgoing_mutex_);
        for (size_t j = local.size(); j > i; --j) {
          outgoing_.push_front(local[j - 1]);
        }
        return false;
      }
    }
    return true;
  }

  bool read_one(bool flush_status)
  {
    while (!take_packet(buffer_, packet_)) {
      if (stop_.load()) {
        return false;
      }
      pollfd pfds[2]{};
      pfds[0].fd = fd_;
      pfds[0].events = POLLIN;
      pfds[1].fd = wake_fd_;
      pfds[1].events = POLLIN;
      const nfds_t count = wake_fd_ >= 0 ? 2 : 1;
      const int rc = ::poll(pfds, count, 200);
      if (rc < 0) {
        if (errno == EINTR) {
          continue;
        }
        return false;
      }
      if (count == 2 && (pfds[1].revents & POLLIN) != 0) {
        drain_wake();
        if (flush_status && !flush_outgoing()) {
          return false;
        }
        last_io_ = std::chrono::steady_clock::now();
      }
      if ((pfds[0].revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
        return false;
      }
      if ((pfds[0].revents & POLLIN) == 0) {
        if (std::chrono::steady_clock::now() - last_io_ > std::chrono::seconds(15)) {
          const std::string ping{static_cast<char>(0xc0), static_cast<char>(0x00)};
          if (!send_all(fd_, ping, stop_)) {
            return false;
          }
          last_io_ = std::chrono::steady_clock::now();
        }
        continue;
      }
      char tmp[512];
      const ssize_t n = ::recv(fd_, tmp, sizeof(tmp), 0);
      if (n == 0) {
        return false;
      }
      if (n < 0) {
        if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) {
          continue;
        }
        return false;
      }
      buffer_.append(tmp, static_cast<size_t>(n));
      last_io_ = std::chrono::steady_clock::now();
    }
    return true;
  }

  bool handle_publish(const MqttTaskClient::Handler & handler)
  {
    last_io_ = std::chrono::steady_clock::now();
    const uint8_t header = static_cast<uint8_t>(packet_[0]);
    const uint8_t type = header >> 4;
    if (type == 13 || type != 3 || packet_.size() < 5) {
      return true;
    }

    size_t offset = 1;
    uint8_t encoded = 0;
    do {
      encoded = static_cast<uint8_t>(packet_[offset++]);
    } while ((encoded & 0x80u) != 0 && offset < packet_.size());

    if (offset + 2 > packet_.size()) {
      return true;
    }
    const uint16_t topic_len = static_cast<uint16_t>(
      (static_cast<uint8_t>(packet_[offset]) << 8) | static_cast<uint8_t>(packet_[offset + 1]));
    offset += 2 + topic_len;
    const uint8_t qos = (header >> 1) & 0x03;
    if (qos > 0) {
      if (offset + 2 > packet_.size()) {
        return true;
      }
      const std::string puback{
        static_cast<char>(0x40), static_cast<char>(0x02), packet_[offset], packet_[offset + 1]};
      if (!send_all(fd_, puback, stop_)) {
        return false;
      }
      offset += 2;
    }
    if (offset > packet_.size()) {
      return true;
    }
    const std::string payload = packet_.substr(offset);
    if (handler && !payload.empty()) {
      handler(payload);
    }
    return true;
  }

  int fd() const { return fd_; }
  const std::string & packet() const { return packet_; }

private:
  void drain_wake()
  {
    if (wake_fd_ < 0) {
      return;
    }
    char tmp[32];
    while (::read(wake_fd_, tmp, sizeof(tmp)) > 0) {
    }
  }

  int fd_;
  std::atomic<bool> & stop_;
  int wake_fd_;
  std::mutex & outgoing_mutex_;
  std::deque<std::string> & outgoing_;
  std::string status_topic_;
  std::string buffer_;
  std::string packet_;
  std::chrono::steady_clock::time_point last_io_;
};

std::string control_packet(uint8_t type, const std::string & body)
{
  std::string packet;
  packet.push_back(static_cast<char>(type));
  append_remaining_length(packet, body.size());
  packet += body;
  return packet;
}

}  // namespace

MqttTaskClient::MqttTaskClient(
  std::string host,
  int port,
  std::string command_topic,
  std::string status_topic,
  Handler handler,
  Logger logger)
: host_(std::move(host)),
  port_(port),
  topic_(std::move(command_topic)),
  status_topic_(std::move(status_topic)),
  handler_(std::move(handler)),
  logger_(std::move(logger))
{
  if (::pipe2(wake_fds_, O_CLOEXEC | O_NONBLOCK) != 0) {
    wake_fds_[0] = -1;
    wake_fds_[1] = -1;
  }
}

MqttTaskClient::~MqttTaskClient()
{
  stop_.store(true);
  wake();
  if (thread_.joinable()) {
    thread_.join();
  }
  if (wake_fds_[0] >= 0) {
    ::close(wake_fds_[0]);
  }
  if (wake_fds_[1] >= 0) {
    ::close(wake_fds_[1]);
  }
}

void MqttTaskClient::start()
{
  if (thread_.joinable()) {
    return;
  }
  thread_ = std::thread([this]() { run(); });
}

void MqttTaskClient::publish_status(const std::string & payload)
{
  {
    std::lock_guard<std::mutex> lock(outgoing_mutex_);
    outgoing_.push_back(payload);
    while (outgoing_.size() > 16) {
      outgoing_.pop_front();
    }
  }
  wake();
}

void MqttTaskClient::wake()
{
  if (wake_fds_[1] < 0) {
    return;
  }
  const char byte = 'x';
  if (::write(wake_fds_[1], &byte, 1) < 0) {
    return;
  }
}

void MqttTaskClient::run()
{
  while (!stop_.load()) {
    if (!session_once() && !stop_.load()) {
      for (int i = 0; i < 20 && !stop_.load(); ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    }
  }
}

bool MqttTaskClient::session_once()
{
  const int fd = connect_broker(host_, port_, stop_, logger_);
  if (fd < 0) {
    return false;
  }
  BrokerConn conn(fd, stop_, wake_fds_[0], outgoing_mutex_, outgoing_, status_topic_);

  std::string body;
  append_u16(body, 4);
  body += "MQTT";
  body.push_back(static_cast<char>(4));
  body.push_back(static_cast<char>(0x02));
  append_u16(body, 30);
  append_u16(body, static_cast<uint16_t>(std::string("sitl-mission").size()));
  body += "sitl-mission";
  if (!send_all(conn.fd(), control_packet(0x10, body), stop_)) {
    return false;
  }
  if (!conn.read_one(false) || conn.packet().size() < 4 ||
    static_cast<uint8_t>(conn.packet()[0]) != 0x20 ||
    static_cast<uint8_t>(conn.packet().back()) != 0x00)
  {
    if (logger_) {
      logger_("MQTT CONNACK 失败");
    }
    return false;
  }

  std::string sub_body;
  append_u16(sub_body, 1);
  append_u16(sub_body, static_cast<uint16_t>(topic_.size()));
  sub_body += topic_;
  sub_body.push_back(static_cast<char>(0));
  if (!send_all(conn.fd(), control_packet(0x82, sub_body), stop_) || !conn.read_one(false) ||
    (static_cast<uint8_t>(conn.packet()[0]) >> 4) != 9)
  {
    if (logger_) {
      logger_("MQTT SUBACK 失败");
    }
    return false;
  }

  if (logger_) {
    logger_("MQTT 已订阅 " + topic_);
  }
  if (!conn.flush_outgoing()) {
    return false;
  }

  while (!stop_.load()) {
    if (!conn.read_one(true) || !conn.handle_publish(handler_)) {
      break;
    }
  }
  return false;
}

}  // namespace sitl_mission_node
