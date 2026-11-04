#include <array>
#include <QApplication>
#include <QCheckBox>
#include <QLabel>
#include <QDoubleSpinBox>
#include <QPushButton>
#include <QTimer>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/float64.hpp>
#include <rviz_common/panel.hpp>
#include <pluginlib/class_loader.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  QApplication app(argc, argv);
  auto server_node = std::make_shared<rclcpp::Node>("draw_s");
  server_node->declare_parameter("error_deg", 0.7);
  server_node->declare_parameter("mpc_enabled", false);
  server_node->declare_parameter("pid_enabled", false);
  server_node->declare_parameter("lqr_enabled", false);
  server_node->declare_parameter("motor_tau", 0.35);
  const std::array<double, 6> initial = {-1, 0.1, 0.2, 0.3, 0.4, 0.5};
  for (size_t i = 0; i < 6; ++i) {
    server_node->declare_parameter("motor_tau_j" + std::to_string(i + 1), initial[i]);
  }
  auto error_pub = server_node->create_publisher<std_msgs::msg::Float64>("/writing/pen_error_mm", 1);
  auto rms_pub = server_node->create_publisher<std_msgs::msg::Float64>("/writing/pen_rms_mm", 1);
  int calls = 0;
  bool correct = true;
  auto server = server_node->create_service<std_srvs::srv::Trigger>("/restart_writing",
    [&](std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr response) {
      const double expected = calls != 1 ? 1.25 : 0.0;
      correct = correct && server_node->get_parameter("error_deg").as_double() == expected;
      correct = correct && server_node->get_parameter("mpc_enabled").as_bool() == (calls == 0 || calls == 2);
      correct = correct && server_node->get_parameter("pid_enabled").as_bool() == (calls == 3);
      correct = correct && server_node->get_parameter("lqr_enabled").as_bool() == (calls == 4);
      correct = correct && server_node->get_parameter("motor_tau").as_double() ==
        (calls != 1 ? 0.5 : 0.0);
      const std::array<double, 6> times = {0.5, 0.5, 0.8, 0.0, 0.5, 0.5};
      for (size_t i = 0; i < 6; ++i) {
        correct = correct && server_node->get_parameter("motor_tau_j" + std::to_string(i + 1))
          .as_double() == (calls != 1 ? times[i] : 0.0);
      }
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
  auto * pid = panel->findChild<QCheckBox *>("pid_enabled_checkbox");
  auto * lqr = panel->findChild<QCheckBox *>("lqr_enabled_checkbox");
  auto * uniform = panel->findChild<QDoubleSpinBox *>("motor_response_seconds");
  auto * fill = panel->findChild<QPushButton *>("set_all_motor_times");
  auto * zero = panel->findChild<QPushButton *>("zero_all_settings");
  auto * initial_restore = panel->findChild<QPushButton *>("restore_initial_settings");
  auto * restore = panel->findChild<QPushButton *>("restore_previous_settings");
  auto * zero_j4 = panel->findChild<QPushButton *>("motor_tau_zero_j4");
  std::array<QDoubleSpinBox *, 6> fields{};
  for (size_t i = 0; i < 6; ++i) {
    fields[i] = panel->findChild<QDoubleSpinBox *>(
      QString::fromStdString("motor_response_j" + std::to_string(i + 1) + "_seconds"));
    if (!fields[i]) {return 1;}
  }
  if (!button || !error || !mpc || !pid || !lqr || !uniform || !fill || !zero || !restore || !initial_restore || !zero_j4 || button->isEnabled()) {return 1;}
  panel->resize(600, 1000);
  panel->show();
  QTimer driver;
  int clicks = 0;
  QObject::connect(&driver, &QTimer::timeout, [&]() {
    rclcpp::spin_some(server_node);
    std_msgs::msg::Float64 reading;
    reading.data = 3.125; error_pub->publish(reading);
    reading.data = 1.25; rms_pub->publish(reading);
    if (button->isEnabled() && calls == clicks) {
      const auto expected_label = calls == 0 ? QString::fromUtf8("开始运行") :
        QString::fromUtf8("重新运行");
      if (button->text() != expected_label) {app.exit(14); return;}
      if (calls == 5) {
        if (!correct) {app.exit(3); return;}
        auto * live_error = panel->findChild<QLabel *>("pen_error_readout");
        auto * live_rms = panel->findChild<QLabel *>("pen_rms_readout");
        if (!live_error || !live_rms || !live_error->text().contains("3.125") ||
          !live_rms->text().contains("1.250")) {app.exit(6); return;}
        if (restore->height() != button->height() || zero->height() != button->height() ||
          initial_restore->height() != button->height() ||
          initial_restore->x() != button->x() ||
          std::abs(initial_restore->width() - restore->width()) > 1 ||
          initial_restore->y() != restore->y() || restore->y() != zero->y() ||
          initial_restore->x() + initial_restore->width() >= restore->x() ||
          zero->x() + zero->width() != button->x() + button->width() ||
          restore->y() <= button->y()) {app.exit(10); return;}
        std::cout << "PASS: six-servo sync, uniform fill, individual edits, row/all zeros, PID/LQR/MPC exclusivity and staged changes, initial/previous restore before/after replay, three-button alignment, atomic replay and error readouts\n";
        app.exit(0);
      } else {
        const double old_error = server_node->get_parameter("error_deg").as_double();
        const double old_j3 = server_node->get_parameter("motor_tau_j3").as_double();
        if (calls == 0) {
          if (error->value() != 0.7 || mpc->isChecked()) {app.exit(4); return;}
          for (size_t i = 0; i < 6; ++i) {
            if (fields[i]->value() != (initial[i] < 0 ? 0.35 : initial[i])) {app.exit(4); return;}
          }
          uniform->setValue(.5); fill->click();
          for (auto * field : fields) {if (field->value() != .5) {app.exit(7); return;}}
          fields[2]->setValue(.8); zero_j4->click();
          error->setValue(1.25); mpc->setChecked(true);
        } else if (calls == 1) {
          zero->click();
          restore->click();
          if (error->value() != 1.25 || uniform->value() != .5 || !mpc->isChecked() ||
            fields[2]->value() != .8 || fields[3]->value() != 0) {app.exit(9); return;}
          zero->click();
          for (auto * field : fields) {if (field->value() != 0) {app.exit(8); return;}}
          if (error->value() != 0 || uniform->value() != 0 || mpc->isChecked()) {app.exit(8); return;}
          // Repeated clearing must preserve the recoverable settings.
          zero->click();
        } else if (calls == 2) {
          initial_restore->click();
          if (error->value() != .7 || uniform->value() != .35 ||
            mpc->isChecked() || pid->isChecked() || lqr->isChecked()) {app.exit(13); return;}
          for (size_t i = 0; i < 6; ++i) {
            if (fields[i]->value() != (initial[i] < 0 ? .35 : initial[i])) {app.exit(13); return;}
          }
          // Initial restore must leave the previous-run history intact.
          restore->click();
          const std::array<double, 6> saved = {.5, .5, .8, 0, .5, .5};
          if (error->value() != 1.25 || uniform->value() != .5 || !mpc->isChecked()) {
            app.exit(9); return;
          }
          for (size_t i = 0; i < 6; ++i) {
            if (fields[i]->value() != saved[i]) {app.exit(9); return;}
          }
        }
        if (calls == 3) {
          pid->click();
          if (!pid->isChecked() || mpc->isChecked() || lqr->isChecked()) {app.exit(11); return;}
          // Clicking the selected option again leaves all three off.
          pid->click();
          if (pid->isChecked() || mpc->isChecked() || lqr->isChecked()) {app.exit(11); return;}
          pid->click();
          if (server_node->get_parameter("pid_enabled").as_bool()) {app.exit(12); return;}
        } else if (calls == 4) {
          lqr->click();
          if (!lqr->isChecked() || mpc->isChecked() || pid->isChecked()) {app.exit(11); return;}
          zero->click();
          if (mpc->isChecked() || pid->isChecked() || lqr->isChecked()) {app.exit(11); return;}
          restore->click();
          if (!lqr->isChecked() || mpc->isChecked() || pid->isChecked()) {app.exit(11); return;}
          if (server_node->get_parameter("lqr_enabled").as_bool()) {app.exit(12); return;}
        }
        if (server_node->get_parameter("error_deg").as_double() != old_error ||
          server_node->get_parameter("motor_tau_j3").as_double() != old_j3) {app.exit(5); return;}
        button->click(); ++clicks;
      }
    }
  });
  driver.start(30);
  QTimer::singleShot(10000, [&]() {app.exit(2);});
  int result = app.exec();
  rclcpp::shutdown();
  return result;
}
