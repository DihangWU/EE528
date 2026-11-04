#ifndef NEXARM_RVIZ_PLUGINS__REPLAY_PANEL_HPP_
#define NEXARM_RVIZ_PLUGINS__REPLAY_PANEL_HPP_

#include <chrono>
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QPushButton>
#include <QTimer>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rviz_common/panel.hpp>
#include <std_srvs/srv/trigger.hpp>

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
  QPushButton * button_;
  QDoubleSpinBox * error_;
  QCheckBox * mpc_;
  QLabel * status_;
  QTimer * timer_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;
  std::shared_ptr<rclcpp::AsyncParametersClient> parameters_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future_;
  std::shared_future<std::vector<rclcpp::Parameter>> sync_future_;
  std::shared_future<rcl_interfaces::msg::SetParametersResult> settings_future_;
  Phase phase_{Phase::Idle};
  bool synced_{false};
  double submitted_error_{0.2};
  bool submitted_mpc_{false};
  int64_t request_id_{0};
  std::chrono::steady_clock::time_point sent_;
};
}  // namespace nexarm_rviz_plugins
#endif
