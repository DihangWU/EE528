#include "nexarm_rviz_plugins/replay_panel.hpp"

#include <QFormLayout>
#include <QSignalBlocker>
#include <QVBoxLayout>
#include <pluginlib/class_list_macros.hpp>

namespace nexarm_rviz_plugins
{
ReplayPanel::ReplayPanel(QWidget * parent) : rviz_common::Panel(parent)
{
  auto * layout = new QVBoxLayout(this);
  auto * form = new QFormLayout();
  error_ = new QDoubleSpinBox(this);
  error_->setObjectName("arrival_error_degrees");
  error_->setRange(0.0, 180.0);
  error_->setDecimals(3);
  error_->setSingleStep(0.1);
  error_->setSuffix(QString::fromUtf8(" °"));
  error_->setPrefix(QString::fromUtf8("±"));
  error_->setValue(0.2);
  error_->setKeyboardTracking(false);
  error_->setToolTip(QString::fromUtf8("每个电机的随机误差上限；0 表示无误差。下次重新运行生效。"));
  mpc_ = new QCheckBox(QString::fromUtf8("启用 MPC"), this);
  mpc_->setObjectName("mpc_enabled_checkbox");
  mpc_->setToolTip("Linear Constrained Incremental MPC");
  form->addRow(QString::fromUtf8("到位误差"), error_);
  form->addRow(QString::fromUtf8("控制优化"), mpc_);
  layout->addLayout(form);
  auto * hint = new QLabel(QString::fromUtf8("修改设置后，点击重新运行生效。"), this);
  hint->setWordWrap(true);
  layout->addWidget(hint);
  button_ = new QPushButton(QString::fromUtf8("重新运行"), this);
  button_->setObjectName("restart_writing_button");
  button_->setMinimumHeight(38);
  button_->setEnabled(false);
  error_->setEnabled(false);
  mpc_->setEnabled(false);
  status_ = new QLabel(QString::fromUtf8("等待写字节点…"), this);
  status_->setWordWrap(true);
  layout->addWidget(button_);
  layout->addWidget(status_);
  layout->addStretch();
  timer_ = new QTimer(this);
  connect(button_, &QPushButton::clicked, this, &ReplayPanel::restart);
  connect(error_, qOverload<double>(&QDoubleSpinBox::valueChanged), this,
    [this](double) {editsChanged();});
  connect(mpc_, &QCheckBox::toggled, this, [this](bool) {editsChanged();});
  connect(timer_, &QTimer::timeout, this, &ReplayPanel::poll);
}

void ReplayPanel::onInitialize()
{
  node_ = std::make_shared<rclcpp::Node>("nexarm_replay_panel");
  client_ = node_->create_client<std_srvs::srv::Trigger>("/restart_writing");
  parameters_ = std::make_shared<rclcpp::AsyncParametersClient>(node_, "/draw_s");
  timer_->start(100);
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
  submitted_error_ = error_->value();
  submitted_mpc_ = mpc_->isChecked();
  settings_future_ = parameters_->set_parameters_atomically({
    rclcpp::Parameter("error_deg", submitted_error_),
    rclcpp::Parameter("mpc_enabled", submitted_mpc_)});
  sent_ = std::chrono::steady_clock::now();
  phase_ = Phase::Applying;
  button_->setEnabled(false);
  error_->setEnabled(false);
  mpc_->setEnabled(false);
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
      if (values.size() != 2) {throw std::runtime_error("Incomplete writing settings");}
      const QSignalBlocker block_error(error_), block_mpc(mpc_);
      error_->setValue(values[0].as_double());
      mpc_->setChecked(values[1].as_bool());
      synced_ = true;
      phase_ = Phase::Idle;
      status_->setText(QString::fromUtf8("设置已读取，点击可从头重播。"));
    } else if (phase_ == Phase::Applying &&
      settings_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
    {
      auto result = settings_future_.get();
      if (!result.successful) {
        phase_ = Phase::Idle;
        status_->setText(QString::fromUtf8("设置未应用：") + QString::fromStdString(result.reason));
      } else {
        sendReplay();
      }
    } else if (phase_ == Phase::Restarting &&
      future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
    {
      auto response = future_.get();
      phase_ = Phase::Idle;
      status_->setText(response->success ?
        QString::fromUtf8("已重播 · 误差 ±%1° · MPC %2")
        .arg(submitted_error_, 0, 'f', 3)
        .arg(submitted_mpc_ ? QString::fromUtf8("开启") : QString::fromUtf8("关闭")) :
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
    status_->setText(QString::fromUtf8("请求超时，请重试；设置仅在成功重播后生效。"));
  }
  if (!ready) {
    synced_ = false;
    if (phase_ == Phase::Idle) {
      status_->setText(QString::fromUtf8("等待写字节点，请启动 draw_s.launch.py。"));
    }
  } else if (!synced_ && phase_ == Phase::Idle) {
    sync_future_ = parameters_->get_parameters({"error_deg", "mpc_enabled"});
    sent_ = std::chrono::steady_clock::now();
    phase_ = Phase::Syncing;
  }
  const bool editable = ready && synced_ && phase_ == Phase::Idle;
  button_->setEnabled(editable);
  error_->setEnabled(editable);
  mpc_->setEnabled(editable);
}
}  // namespace nexarm_rviz_plugins

PLUGINLIB_EXPORT_CLASS(nexarm_rviz_plugins::ReplayPanel, rviz_common::Panel)
