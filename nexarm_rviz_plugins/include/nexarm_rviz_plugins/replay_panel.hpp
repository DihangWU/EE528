#ifndef NEXARM_RVIZ_PLUGINS__REPLAY_PANEL_HPP_
#define NEXARM_RVIZ_PLUGINS__REPLAY_PANEL_HPP_

#include <array>
#include <chrono>
#include <vector>
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QPushButton>
#include <QTimer>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rviz_common/panel.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/float64.hpp>

namespace nexarm_rviz_plugins
{
class ReplayPanel : public rviz_common::Panel
{
  Q_OBJECT
public:
  explicit ReplayPanel(QWidget * parent = nullptr);
  void onInitialize() override;

private:
  enum class Phase {Idle, Syncing, Applying, Restarting};
  void restart();
  void poll();
  void editsChanged();
  void sendReplay();
  void setAllTimes(double value);
  void zeroSettings();
  struct Settings
  {
    double error{0.2};
    double uniform{0.2};
    std::array<double, 6> taus{};
    bool mpc{false};
  };
  Settings readSettings();
  void restoreSettings();
  Settings active_settings_;
  Settings previous_settings_;
  Settings submitted_settings_;
  void enableEditors(bool enabled);
  QPushButton * button_;
  QDoubleSpinBox * error_;
  QDoubleSpinBox * motor_tau_;
  std::array<QDoubleSpinBox *, 6> motor_taus_;
  std::vector<QWidget *> editors_;
  QCheckBox * mpc_;
  QLabel * status_;
  QLabel * tracking_error_;
  QLabel * tracking_rms_;
  QTimer * timer_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;
  std::shared_ptr<rclcpp::AsyncParametersClient> parameters_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr error_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr rms_subscription_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future_;
  std::shared_future<std::vector<rclcpp::Parameter>> sync_future_;
  std::shared_future<rcl_interfaces::msg::SetParametersResult> settings_future_;
  Phase phase_{Phase::Idle};
  bool synced_{false};
  double submitted_error_{0.2};
  std::array<double, 6> submitted_taus_{};
  bool submitted_mpc_{false};
  int64_t request_id_{0};
  std::chrono::steady_clock::time_point sent_;
};
}  // namespace nexarm_rviz_plugins
#endif
