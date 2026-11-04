#include <QApplication>
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QPushButton>
#include <QTimer>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <rviz_common/panel.hpp>
#include <pluginlib/class_loader.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  QApplication app(argc, argv);
  auto server_node = std::make_shared<rclcpp::Node>("draw_s");
  server_node->declare_parameter("error_deg", 0.7);
  server_node->declare_parameter("mpc_enabled", false);
  int calls = 0;
  bool correct = true;
  auto server = server_node->create_service<std_srvs::srv::Trigger>("/restart_writing",
    [&](std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr response) {
      const double expected = calls == 0 ? 1.25 : 0.0;
      correct = correct && server_node->get_parameter("error_deg").as_double() == expected;
      correct = correct && server_node->get_parameter("mpc_enabled").as_bool() == (calls == 0);
      ++calls;
      response->success = correct;
      response->message = "restarted";
    });
  pluginlib::ClassLoader<rviz_common::Panel> loader("rviz_common", "rviz_common::Panel");
  auto panel = loader.createSharedInstance("nexarm_rviz_plugins/ReplayPanel");
  panel->onInitialize();
  auto * button = panel->findChild<QPushButton *>("restart_writing_button");
  auto * error = panel->findChild<QDoubleSpinBox *>("arrival_error_degrees");
  auto * mpc = panel->findChild<QCheckBox *>("mpc_enabled_checkbox");
  if (!button || !error || !mpc || button->isEnabled()) {return 1;}
  QTimer driver;
  int clicks = 0;
  QObject::connect(&driver, &QTimer::timeout, [&]() {
    rclcpp::spin_some(server_node);
    if (button->isEnabled() && calls == clicks) {
      if (calls == 2) {
        if (!correct) {app.exit(3); return;}
        std::cout << "PASS: plugin loaded, initial settings synced, edits staged, atomic settings applied before two replays\n";
        app.exit(0);
      } else {
        if (calls == 0 && (error->value() != 0.7 || mpc->isChecked())) {
          app.exit(4); return;
        }
        const double old_error = server_node->get_parameter("error_deg").as_double();
        error->setValue(calls == 0 ? 1.25 : 0.0);
        mpc->setChecked(calls == 0);
        // Widget edits alone must not change ROS settings.
        if (server_node->get_parameter("error_deg").as_double() != old_error) {
          app.exit(5); return;
        }
        button->click();
        ++clicks;
      }
    }
  });
  driver.start(30);
  QTimer::singleShot(10000, [&]() {app.exit(2);});
  int result = app.exec();
  rclcpp::shutdown();
  return result;
}
