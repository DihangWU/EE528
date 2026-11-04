#include "nexarm_rviz_plugins/replay_panel.hpp"

#include <QFormLayout>
#include <QHBoxLayout>
#include <QSignalBlocker>
#include <QScrollArea>
#include <QStringList>
#include <QVBoxLayout>
#include <pluginlib/class_list_macros.hpp>

namespace nexarm_rviz_plugins
{
ReplayPanel::ReplayPanel(QWidget * parent) : rviz_common::Panel(parent)
{
  auto * outer = new QVBoxLayout(this);
  outer->setContentsMargins(0, 0, 0, 0);
  auto * scroll = new QScrollArea(this);
  scroll->setWidgetResizable(true);
  auto * content = new QWidget(scroll);
  auto * layout = new QVBoxLayout(content);
  scroll->setWidget(content);
  outer->addWidget(scroll);
  auto * form = new QFormLayout();
  auto make_time = [this](const QString & name) {
      auto * field = new QDoubleSpinBox(this);
      field->setObjectName(name);
      field->setRange(0.0, 60.0);
      field->setDecimals(3);
      field->setSingleStep(0.05);
      field->setSuffix(QString::fromUtf8(" 秒"));
      field->setValue(0.2);
      field->setKeyboardTracking(false);
      field->setToolTip(QString::fromUtf8("一阶响应时间常数 τ，0 关闭滞后。不是纯等待延迟；重新运行后生效。"));
      editors_.push_back(field);
      connect(field, qOverload<double>(&QDoubleSpinBox::valueChanged), this,
        [this](double) {editsChanged();});
      return field;
    };
  auto make_row = [this](QDoubleSpinBox * field, const QString & zero_name) {
      auto * row = new QWidget(this);
      auto * line = new QHBoxLayout(row);
      line->setContentsMargins(0, 0, 0, 0);
      line->addWidget(field);
      auto * zero = new QPushButton(QString::fromUtf8("归零"), row);
      zero->setObjectName(zero_name);
      editors_.push_back(zero);
      connect(zero, &QPushButton::clicked, this, [field]() {field->setValue(0);});
      line->addWidget(zero);
      return row;
    };
  error_ = new QDoubleSpinBox(this);
  error_->setObjectName("arrival_error_degrees");
  error_->setRange(0.0, 180.0);
  error_->setDecimals(3);
  error_->setSingleStep(0.1);
  error_->setSuffix(QString::fromUtf8(" °"));
  error_->setPrefix(QString::fromUtf8("±"));
  error_->setValue(0.2);
  error_->setKeyboardTracking(false);
  error_->setToolTip(QString::fromUtf8("J1–J5 随机误差上限；0 仅关闭随机误差。J6 保持夹持。"));
  editors_.push_back(error_);
  connect(error_, qOverload<double>(&QDoubleSpinBox::valueChanged), this,
    [this](double) {editsChanged();});
  form->addRow(QString::fromUtf8("到位误差"), make_row(error_, "arrival_error_zero"));

  motor_tau_ = make_time("motor_response_seconds");
  auto * uniform_row = new QWidget(this);
  auto * uniform_line = new QHBoxLayout(uniform_row);
  uniform_line->setContentsMargins(0, 0, 0, 0);
  uniform_line->addWidget(motor_tau_);
  auto * uniform = new QPushButton(QString::fromUtf8("统一到六项"), uniform_row);
  uniform->setObjectName("set_all_motor_times");
  editors_.push_back(uniform);
  connect(uniform, &QPushButton::clicked, this, [this]() {
      motor_tau_->interpretText();
      setAllTimes(motor_tau_->value());
    });
  uniform_line->addWidget(uniform);
  auto * zero_times = new QPushButton(QString::fromUtf8("归零"), uniform_row);
  zero_times->setObjectName("zero_all_motor_times");
  editors_.push_back(zero_times);
  connect(zero_times, &QPushButton::clicked, this, [this]() {
      motor_tau_->setValue(0); setAllTimes(0);
    });
  uniform_line->addWidget(zero_times);
  form->addRow(QString::fromUtf8("统一响应 τ"), uniform_row);
  for (size_t i = 0; i < motor_taus_.size(); ++i) {
    const QString number = QString::number(i + 1);
    motor_taus_[i] = make_time("motor_response_j" + number + "_seconds");
    const QString label = i == 5 ? QString::fromUtf8("J6 夹爪 τ") : "J" + number + " τ";
    form->addRow(label, make_row(motor_taus_[i], "motor_tau_zero_j" + number));
  }
  mpc_ = new QCheckBox(QString::fromUtf8("启用 MPC"), this);
  mpc_->setObjectName("mpc_enabled_checkbox");
  mpc_->setToolTip("Linear Constrained Incremental MPC");
  editors_.push_back(mpc_);
  connect(mpc_, &QCheckBox::toggled, this, [this](bool) {editsChanged();});
  form->addRow(QString::fromUtf8("控制优化"), mpc_);
  layout->addLayout(form);
  auto * hint = new QLabel(QString::fromUtf8("统一填写或单独修改，点击重新运行后生效。归零只修改待用参数。"), this);
  hint->setWordWrap(true);
  layout->addWidget(hint);
  auto * zero_all = new QPushButton(QString::fromUtf8("参数全部归零"), this);
  zero_all->setObjectName("zero_all_settings");
  auto zero_font = zero_all->font();
  zero_font.setPointSizeF(9.0);
  zero_all->setFont(zero_font);
  zero_all->setFixedWidth(116);
  zero_all->setFixedHeight(38);
  zero_all->setAutoDefault(false);
  zero_all->setToolTip(QString::fromUtf8("将待用参数全部置 0；点击重新运行后应用。"));
  editors_.push_back(zero_all);
  connect(zero_all, &QPushButton::clicked, this, &ReplayPanel::zeroSettings);
  auto * restore = new QPushButton(QString::fromUtf8("恢复上一次参数"), this);
  restore->setObjectName("restore_previous_settings");
  restore->setFixedHeight(38);
  restore->setAutoDefault(false);
  restore->setToolTip(QString::fromUtf8("恢复归零前或上一次运行的参数；点击重新运行后应用。"));
  editors_.push_back(restore);
  connect(restore, &QPushButton::clicked, this, &ReplayPanel::restoreSettings);
  button_ = new QPushButton(QString::fromUtf8("重新运行"), this);
  button_->setObjectName("restart_writing_button");
  button_->setFixedHeight(38);
  button_->setEnabled(false);
  enableEditors(false);
  status_ = new QLabel(QString::fromUtf8("等待写字节点…"), this);
  status_->setWordWrap(true);
  layout->addWidget(button_);
  auto * reset_row = new QHBoxLayout();
  reset_row->setContentsMargins(0, 8, 0, 2);
  reset_row->addWidget(restore, 1);
  reset_row->addWidget(zero_all);
  layout->addLayout(reset_row);
  layout->addWidget(status_);
  tracking_error_ = new QLabel(QString::fromUtf8("实时跟踪误差：— mm"), this);
  tracking_error_->setObjectName("pen_error_readout");
  tracking_rms_ = new QLabel(QString::fromUtf8("本次落笔 RMS：— mm"), this);
  tracking_rms_->setObjectName("pen_rms_readout");
  tracking_error_->setToolTip(QString::fromUtf8("实际笔尖与同一时刻目标笔尖的三维距离，包含时间滞后。"));
  tracking_rms_->setToolTip(QString::fromUtf8("本次落笔采样点跟踪误差 RMS，重播清零；不是轮廓误差。"));
  layout->addWidget(tracking_error_);
  layout->addWidget(tracking_rms_);
  layout->addStretch();
  timer_ = new QTimer(this);
  connect(button_, &QPushButton::clicked, this, &ReplayPanel::restart);
  connect(timer_, &QTimer::timeout, this, &ReplayPanel::poll);
}

void ReplayPanel::onInitialize()
{
  node_ = std::make_shared<rclcpp::Node>("nexarm_replay_panel");
  client_ = node_->create_client<std_srvs::srv::Trigger>("/restart_writing");
  parameters_ = std::make_shared<rclcpp::AsyncParametersClient>(node_, "/draw_s");
  error_subscription_ = node_->create_subscription<std_msgs::msg::Float64>(
    "/writing/pen_error_mm", 1, [this](std_msgs::msg::Float64::ConstSharedPtr message) {
      tracking_error_->setText(QString::fromUtf8("实时跟踪误差：%1 mm").arg(message->data, 0, 'f', 3));
    });
  rms_subscription_ = node_->create_subscription<std_msgs::msg::Float64>(
    "/writing/pen_rms_mm", 1, [this](std_msgs::msg::Float64::ConstSharedPtr message) {
      tracking_rms_->setText(QString::fromUtf8("本次落笔 RMS：%1 mm").arg(message->data, 0, 'f', 3));
    });
  timer_->start(100);
}

void ReplayPanel::enableEditors(bool enabled)
{
  for (auto * editor : editors_) {editor->setEnabled(enabled);}
}

void ReplayPanel::setAllTimes(double value)
{
  for (auto * field : motor_taus_) {field->setValue(value);}
  editsChanged();
}

ReplayPanel::Settings ReplayPanel::readSettings()
{
  Settings settings;
  error_->interpretText();
  motor_tau_->interpretText();
  settings.error = error_->value();
  settings.uniform = motor_tau_->value();
  settings.mpc = mpc_->isChecked();
  for (size_t i = 0; i < motor_taus_.size(); ++i) {
    motor_taus_[i]->interpretText();
    settings.taus[i] = motor_taus_[i]->value();
  }
  return settings;
}

void ReplayPanel::restoreSettings()
{
  error_->setValue(previous_settings_.error);
  motor_tau_->setValue(previous_settings_.uniform);
  for (size_t i = 0; i < motor_taus_.size(); ++i) {
    motor_taus_[i]->setValue(previous_settings_.taus[i]);
  }
  mpc_->setChecked(previous_settings_.mpc);
  status_->setText(QString::fromUtf8("已恢复上一次参数，点击重新运行后应用。"));
}

void ReplayPanel::zeroSettings()
{
  const auto current = readSettings();
  bool nonzero = current.error != 0 || current.uniform != 0 || current.mpc;
  for (double tau : current.taus) {nonzero = nonzero || tau != 0;}
  if (nonzero) {previous_settings_ = current;}
  error_->setValue(0);
  motor_tau_->setValue(0);
  setAllTimes(0);
  mpc_->setChecked(false);
  editsChanged();
}

void ReplayPanel::editsChanged()
{
  if (synced_ && phase_ == Phase::Idle) {
    status_->setText(QString::fromUtf8("设置已修改，将在下次重新运行时应用。"));
  }
}

void ReplayPanel::restart()
{
  if (!synced_ || phase_ != Phase::Idle || !client_->service_is_ready() ||
    !parameters_->service_is_ready()) {return;}
  error_->interpretText();
  motor_tau_->interpretText();
  submitted_settings_ = readSettings();
  submitted_error_ = error_->value();
  submitted_mpc_ = mpc_->isChecked();
  std::vector<rclcpp::Parameter> settings = {
    rclcpp::Parameter("error_deg", submitted_error_),
    rclcpp::Parameter("motor_tau", motor_tau_->value()),
    rclcpp::Parameter("mpc_enabled", submitted_mpc_)};
  for (size_t i = 0; i < motor_taus_.size(); ++i) {
    motor_taus_[i]->interpretText();
    submitted_taus_[i] = motor_taus_[i]->value();
    settings.emplace_back("motor_tau_j" + std::to_string(i + 1), submitted_taus_[i]);
  }
  settings_future_ = parameters_->set_parameters_atomically(settings);
  sent_ = std::chrono::steady_clock::now();
  phase_ = Phase::Applying;
  button_->setEnabled(false);
  enableEditors(false);
  status_->setText(QString::fromUtf8("正在提交设置…"));
}

void ReplayPanel::sendReplay()
{
  auto result = client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  request_id_ = result.request_id;
  future_ = result.future.share();
  sent_ = std::chrono::steady_clock::now();
  phase_ = Phase::Restarting;
  status_->setText(QString::fromUtf8("正在重新开始…"));
}

void ReplayPanel::poll()
{
  if (!node_ || !rclcpp::ok()) {return;}
  rclcpp::spin_some(node_);
  const bool ready = client_->service_is_ready() && parameters_->service_is_ready();
  try {
    if (phase_ == Phase::Syncing &&
      sync_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
    {
      auto values = sync_future_.get();
      if (values.size() != 9) {throw std::runtime_error("Incomplete writing settings");}
      const QSignalBlocker block_error(error_), block_mpc(mpc_), block_tau(motor_tau_);
      error_->setValue(values[0].as_double());
      mpc_->setChecked(values[1].as_bool());
      const double uniform = values[2].as_double();
      motor_tau_->setValue(uniform);
      for (size_t i = 0; i < motor_taus_.size(); ++i) {
        const QSignalBlocker block(motor_taus_[i]);
        const double value = values[3 + i].as_double();
        motor_taus_[i]->setValue(value < 0 ? uniform : value);
      }
      active_settings_ = readSettings();
      previous_settings_ = active_settings_;
      synced_ = true;
      phase_ = Phase::Idle;
      status_->setText(QString::fromUtf8("六舵机设置已读取，点击可从头重播。"));
    } else if (phase_ == Phase::Applying &&
      settings_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
    {
      auto result = settings_future_.get();
      if (!result.successful) {
        phase_ = Phase::Idle;
        status_->setText(QString::fromUtf8("设置未应用：") + QString::fromStdString(result.reason));
      } else {sendReplay();}
    } else if (phase_ == Phase::Restarting &&
      future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
    {
      auto response = future_.get();
      if (response->success) {
        previous_settings_ = active_settings_;
        active_settings_ = submitted_settings_;
      }
      phase_ = Phase::Idle;
      QStringList times;
      for (double value : submitted_taus_) {times << QString::number(value, 'f', 3);}
      status_->setText(response->success ?
        QString::fromUtf8("已重播 · 误差 ±%1° · MPC %2\nJ1–J6 τ：%3 s")
        .arg(submitted_error_, 0, 'f', 3)
        .arg(submitted_mpc_ ? QString::fromUtf8("开启") : QString::fromUtf8("关闭"))
        .arg(times.join(", ")) :
        QString::fromUtf8("重播失败：") + QString::fromStdString(response->message));
    }
  } catch (const std::exception & error) {
    phase_ = Phase::Idle;
    status_->setText(QString::fromUtf8("操作失败：") + QString::fromUtf8(error.what()));
  }
  if (phase_ != Phase::Idle &&
    std::chrono::steady_clock::now() - sent_ > std::chrono::seconds(3))
  {
    if (phase_ == Phase::Restarting) {client_->remove_pending_request(request_id_);}
    phase_ = Phase::Idle;
    status_->setText(QString::fromUtf8("请求超时，请检查状态后重试。"));
  }
  if (!ready) {
    synced_ = false;
    if (phase_ == Phase::Idle) {
      status_->setText(QString::fromUtf8("等待写字节点，请启动 draw_s.launch.py。"));
    }
  } else if (!synced_ && phase_ == Phase::Idle) {
    std::vector<std::string> names = {"error_deg", "mpc_enabled", "motor_tau"};
    for (size_t i = 1; i <= 6; ++i) {names.push_back("motor_tau_j" + std::to_string(i));}
    sync_future_ = parameters_->get_parameters(names);
    sent_ = std::chrono::steady_clock::now();
    phase_ = Phase::Syncing;
  }
  const bool editable = ready && synced_ && phase_ == Phase::Idle;
  button_->setEnabled(editable);
  enableEditors(editable);
}
}  // namespace nexarm_rviz_plugins

PLUGINLIB_EXPORT_CLASS(nexarm_rviz_plugins::ReplayPanel, rviz_common::Panel)
