#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rmw/qos_profiles.h"

using namespace std::chrono_literals;

enum QoSProfile
{
  DEFAULT = 0,
  SYSTEM_DEFAULT,
  SENSOR_DATA,
  SERVICES_DEFAULT,
  PARAMETERS,
  PARAMETER_EVENTS
};

rmw_qos_profile_t get_qos_profile(int qos_profile)
{
  switch (qos_profile)
  {
  case QoSProfile::DEFAULT:
    return rmw_qos_profile_default;
  case QoSProfile::SYSTEM_DEFAULT:
    return rmw_qos_profile_system_default;
  case QoSProfile::SENSOR_DATA:
    return rmw_qos_profile_sensor_data;
  case QoSProfile::SERVICES_DEFAULT:
    return rmw_qos_profile_services_default;
  case QoSProfile::PARAMETERS:
    return rmw_qos_profile_parameters;
  case QoSProfile::PARAMETER_EVENTS:
    return rmw_qos_profile_parameter_events;
  default:
    return rmw_qos_profile_default;
  }
}

std::string get_qos_profile_name(int qos_profile)
{
  switch (qos_profile)
  {
  case QoSProfile::DEFAULT:
    return "default";
  case QoSProfile::SYSTEM_DEFAULT:
    return "system_default";
  case QoSProfile::SENSOR_DATA:
    return "sensor_data";
  case QoSProfile::SERVICES_DEFAULT:
    return "services_default";
  case QoSProfile::PARAMETERS:
    return "parameters";
  case QoSProfile::PARAMETER_EVENTS:
    return "parameter_events";
  default:
    return "default";
  }
}

class Monitor : public rclcpp::Node
{
public:
  Monitor(std::string topic_name) : Node("monitor")
  {
    // Parameters
    int time_window, qos_profile;
    get_params(time_window, qos_profile);
    init_monitor(time_window);

    // Topic
    std::string topic_type;
    if (!init_topic(topic_name, topic_type))
      return;

    // Subscription
    rclcpp::QoS qos(rclcpp::KeepLast(1), get_qos_profile(qos_profile));
    sub_ = create_generic_subscription(
        topic_name,
        topic_type,
        qos,
        [this](std::shared_ptr<rclcpp::SerializedMessage> msg)
        {
          // ************************************************
          // Mutex locked (Counting message)
          while (mutex_.exchange(true, std::memory_order_acquire))
            ;
          count_[head_] += 1;
          byte_size_[head_] += msg->size();
          // Mutex unlocked
          mutex_.store(false, std::memory_order_release);
          // ************************************************
        });

    // Timer
    timer_ = create_wall_timer(
        1s,
        [this]()
        {
          size_t s = count_.size();
          // ************************************************
          // Mutex locked (Get and increase index)
          while (mutex_.exchange(true, std::memory_order_acquire))
            ;
          size_t prev_head = head_;
          head_ = (prev_head + 1) % s;
          // reset
          count_[head_] = 0;
          byte_size_[head_] = 0;
          // Mutex unlocked
          mutex_.store(false, std::memory_order_release);
          // ************************************************
          count_sum_ += count_[prev_head];
          byte_size_sum_ += byte_size_[prev_head];

          float hz = (float)count_sum_ / time_window_f;
          float bw = (float)byte_size_sum_ / time_window_f;
          if (nth_received_ >= time_window_i)
            RCLCPP_INFO(this->get_logger(), "(window: %d [s]) hz: %.1f, bw: %.1f [Byte/s]", time_window_i, hz, bw);
          else
          {
            nth_received_ += 1;
            RCLCPP_WARN(this->get_logger(), "(window: %d [s]) hz: %.1f, bw: %.1f [Byte/s] <- [Invalid] Data too small for the window.", time_window_i, hz, bw);
          }

          count_sum_ -= count_[tail_];
          byte_size_sum_ -= byte_size_[tail_];

          // Update tail
          tail_ = (tail_ + 1) % s;
        });
  }

private:
  std::shared_ptr<rclcpp::GenericSubscription> sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // common
  float time_window_f; // sec
  int time_window_i;   // sec
  int nth_received_;   // n-th received
  size_t head_, tail_;
  std::atomic<bool> mutex_;
  // hz
  std::vector<unsigned int> count_;
  unsigned int count_sum_;
  // bw
  std::vector<size_t> byte_size_;
  size_t byte_size_sum_;

  void get_params(int &time_window, int &qos_profile)
  {
    this->declare_parameter("time_window", 1);
    this->declare_parameter("qos", 0);
    time_window = this->get_parameter("time_window").as_int();
    qos_profile = this->get_parameter("qos").as_int();
    if (time_window < 1)
      time_window = 1;
    RCLCPP_INFO(this->get_logger(), "time_window: %d", time_window);
    RCLCPP_INFO(this->get_logger(), "qos_profile: %d (%s)", qos_profile, get_qos_profile_name(qos_profile).c_str());
  }

  void init_monitor(int time_window)
  {
    // Reset index
    nth_received_ = 0;
    time_window_i = time_window;
    time_window_f = (float)time_window;
    head_ = time_window - 1;
    tail_ = 0;
    mutex_.store(false, std::memory_order_release);
    // Reset vector
    const int buf_size = 1;
    count_.resize(time_window + buf_size);
    byte_size_.resize(time_window + buf_size);
    for (auto &e : count_)
      e = 0;
    for (auto &e : byte_size_)
      e = 0;
    count_sum_ = 0;
    byte_size_sum_ = 0;
  }

  bool init_topic(std::string topic_name, std::string &topic_type)
  {
    // Check topic name
    RCLCPP_INFO(this->get_logger(), "topic_name: %s", topic_name.c_str());
    if (topic_name.empty())
    {
      RCLCPP_ERROR(this->get_logger(), "topic_name is empty");
      return false;
    }
    // Get topic type
    if (!get_topic_type(topic_name, topic_type))
      return false;
    RCLCPP_INFO(this->get_logger(), "topic_type: %s", topic_type.c_str());
    return true;
  }

  bool get_topic_type(std::string topic_name, std::string &topic_type)
  {
    int n_wait = 10;
    if (!wait_for_topic(n_wait, topic_name))
      return false;
    std::map<std::string, std::vector<std::string>> topic_names_and_types = this->get_topic_names_and_types();
    topic_type = topic_names_and_types[topic_name][0];
    return true;
  }

  bool wait_for_topic(int n, std::string topic_name)
  {
    RCLCPP_INFO(this->get_logger(), "Waiting %d sec for topic...", n);
    auto exit_condition = [this, topic_name]()
    {
      std::map<std::string, std::vector<std::string>> topic_names_and_types = this->get_topic_names_and_types();
      return topic_names_and_types.find(topic_name) != topic_names_and_types.end();
    };
    if (!wait_n_sec(n, exit_condition))
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to find topic");
      RCLCPP_WARN(this->get_logger(), "Available topics:");
      for (auto const &topic : this->get_topic_names_and_types())
      {
        RCLCPP_WARN(this->get_logger(), "topic: %s", topic.first.c_str());
        for (auto const &type : topic.second)
        {
          RCLCPP_WARN(this->get_logger(), "  type: %s", type.c_str());
        }
      }
      return false;
    }
    return true;
  }

  bool wait_n_sec(int n, std::function<bool()> exit_condition)
  {
    auto start = std::chrono::system_clock::now();
    while (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - start).count() < n)
    {
      if (exit_condition())
        return true;
      std::this_thread::sleep_for(500ms);
      RCLCPP_INFO(this->get_logger(), ".");
    }
    return false;
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Monitor>(argv[1]));
  rclcpp::shutdown();
  return 0;
}
