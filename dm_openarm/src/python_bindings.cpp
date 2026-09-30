#include "dm_openarm/config.hpp"
#include "dm_openarm/dm_arm.hpp"
#include "dm_openarm/mit_loop_controller.hpp"
#include "dm_openarm/types.hpp"
#include "dm_openarm/yaml_loader.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include <filesystem>
#include <string>

namespace nb = nanobind;

NB_MODULE(_core, m)
{
  m.doc() = "dm_openarm Python bindings";
  m.attr("DEFAULT_CONTROL_HZ") = dm_openarm::kDefaultControlHz;

  nb::enum_<dm_openarm::MotorModel>(m, "MotorModel")
    .value("DM4310", dm_openarm::MotorModel::DM4310)
    .value("DM8009", dm_openarm::MotorModel::DM8009)
    .value("DM4340P", dm_openarm::MotorModel::DM4340P)
    .export_values();

  nb::enum_<dm_openarm::ControlMode>(m, "ControlMode")
    .value("MIT", dm_openarm::ControlMode::MIT)
    .export_values();

  nb::class_<dm_openarm::MitCommand>(m, "MitCommand")
    .def(
      nb::init<double, double, double, double, double>(),
      nb::arg("kp") = 0.0,
      nb::arg("kd") = 0.0,
      nb::arg("q") = 0.0,
      nb::arg("dq") = 0.0,
      nb::arg("tau") = 0.0)
    .def_rw("kp", &dm_openarm::MitCommand::kp)
    .def_rw("kd", &dm_openarm::MitCommand::kd)
    .def_rw("q", &dm_openarm::MitCommand::q)
    .def_rw("dq", &dm_openarm::MitCommand::dq)
    .def_rw("tau", &dm_openarm::MitCommand::tau);

  nb::class_<dm_openarm::MotorState>(m, "MotorState")
    .def_ro("can_id", &dm_openarm::MotorState::can_id)
    .def_ro("mst_id", &dm_openarm::MotorState::mst_id)
    .def_ro("position", &dm_openarm::MotorState::position)
    .def_ro("velocity", &dm_openarm::MotorState::velocity)
    .def_ro("torque", &dm_openarm::MotorState::torque)
    .def_ro("feedback_hz", &dm_openarm::MotorState::feedback_hz)
    .def_ro("feedback_interval_s", &dm_openarm::MotorState::feedback_interval_s)
    .def_ro("last_rx_age_s", &dm_openarm::MotorState::last_rx_age_s)
    .def_ro("rx_sequence", &dm_openarm::MotorState::rx_sequence)
    .def_ro("error_code", &dm_openarm::MotorState::error_code)
    .def_ro("raw_status", &dm_openarm::MotorState::raw_status)
    .def_ro("enabled_confirmed", &dm_openarm::MotorState::enabled_confirmed)
    .def("feedback_fresh", &dm_openarm::MotorState::feedback_fresh,
         nb::arg("max_age_s") = 0.1);

  nb::class_<dm_openarm::MotorTimingStats>(m, "MotorTimingStats")
    .def_ro("can_id", &dm_openarm::MotorTimingStats::can_id)
    .def_ro("rx_hz", &dm_openarm::MotorTimingStats::rx_hz)
    .def_ro("rx_interval_s", &dm_openarm::MotorTimingStats::rx_interval_s)
    .def_ro("last_rx_age_s", &dm_openarm::MotorTimingStats::last_rx_age_s)
    .def_ro("rx_frames", &dm_openarm::MotorTimingStats::rx_frames);

  nb::class_<dm_openarm::TimingStats>(m, "TimingStats")
    .def_ro("connected", &dm_openarm::TimingStats::connected)
    .def_ro("running", &dm_openarm::TimingStats::running)
    .def_ro("window_s", &dm_openarm::TimingStats::window_s)
    .def_ro("target_tx_hz", &dm_openarm::TimingStats::target_tx_hz)
    .def_ro("actual_tx_hz", &dm_openarm::TimingStats::actual_tx_hz)
    .def_ro("tx_cycles", &dm_openarm::TimingStats::tx_cycles)
    .def_ro("deadline_misses", &dm_openarm::TimingStats::deadline_misses)
    .def_ro("motors", &dm_openarm::TimingStats::motors);

  nb::class_<dm_openarm::MotorConfig>(m, "MotorConfig")
    .def(nb::init<>())
    .def_rw("name", &dm_openarm::MotorConfig::name)
    .def_rw("model", &dm_openarm::MotorConfig::model)
    .def_rw("mode", &dm_openarm::MotorConfig::mode)
    .def_rw("can_id", &dm_openarm::MotorConfig::can_id)
    .def_rw("mst_id", &dm_openarm::MotorConfig::mst_id);

  nb::class_<dm_openarm::LimbSpec>(m, "LimbSpec")
    .def(nb::init<>())
    .def_rw("begin", &dm_openarm::LimbSpec::begin)
    .def_rw("count", &dm_openarm::LimbSpec::count);

  nb::class_<dm_openarm::ArmConfig>(m, "ArmConfig")
    .def(nb::init<>())
    .def_rw("usb_serial", &dm_openarm::ArmConfig::usb_serial)
    .def_rw("nom_baud", &dm_openarm::ArmConfig::nom_baud)
    .def_rw("dat_baud", &dm_openarm::ArmConfig::dat_baud)
    .def_rw("canfd", &dm_openarm::ArmConfig::canfd)
    .def_rw("brs", &dm_openarm::ArmConfig::brs)
    .def_rw("device_index", &dm_openarm::ArmConfig::device_index)
    .def_rw("motors", &dm_openarm::ArmConfig::motors)
    .def_rw("bus_motors", &dm_openarm::ArmConfig::bus_motors)
    .def_rw("left", &dm_openarm::ArmConfig::left)
    .def_rw("right", &dm_openarm::ArmConfig::right);

  m.def(
    "load_arm_config",
    [](const std::string& path) {
      return dm_openarm::load_arm_config(std::filesystem::path(path));
    },
    nb::arg("path"));

  nb::class_<dm_openarm::DmArm>(m, "DmArm")
    .def(nb::init<dm_openarm::ArmConfig>())
    .def("connect", &dm_openarm::DmArm::connect)
    .def("enable", &dm_openarm::DmArm::enable, nb::call_guard<nb::gil_scoped_release>())
    .def("disable", &dm_openarm::DmArm::disable, nb::call_guard<nb::gil_scoped_release>())
    .def("disconnect", &dm_openarm::DmArm::disconnect, nb::call_guard<nb::gil_scoped_release>())
    .def("connected", &dm_openarm::DmArm::connected)
    .def("probe_status", &dm_openarm::DmArm::probe_status, nb::arg("timeout_s"),
         nb::call_guard<nb::gil_scoped_release>())
    .def("send_mit_all", &dm_openarm::DmArm::send_mit_all, nb::arg("commands"))
    .def("send_zero_mit_all", &dm_openarm::DmArm::send_zero_mit_all)
    .def(
      "set_zero",
      &dm_openarm::DmArm::set_zero,
      nb::arg("can_id"),
      nb::arg("persist") = true)
    .def("set_zero_all", &dm_openarm::DmArm::set_zero_all, nb::arg("persist") = true)
    .def("states", &dm_openarm::DmArm::states, nb::call_guard<nb::gil_scoped_release>());

  nb::class_<dm_openarm::MitLoopController>(m, "MitLoopController")
    .def(nb::init<dm_openarm::DmArm&>(), nb::keep_alive<1, 2>())
    .def("start", &dm_openarm::MitLoopController::start,
         nb::arg("hz") = dm_openarm::kDefaultControlHz)
    .def("enable_seeded", &dm_openarm::MitLoopController::enable_seeded,
         nb::arg("gains"), nb::arg("hz")=250.0, nb::arg("command_timeout")=0.1, nb::arg("feedback_timeout")=0.2,
         nb::call_guard<nb::gil_scoped_release>())
    .def("start_seeded", &dm_openarm::MitLoopController::start_seeded,
         nb::arg("gains"), nb::arg("hz")=250.0, nb::arg("command_timeout")=0.1, nb::arg("feedback_timeout")=0.2)
    .def("hold", &dm_openarm::MitLoopController::hold, nb::arg("reset_fault")=false)
    .def("hold_command", &dm_openarm::MitLoopController::hold_command)
    .def("safety_state", &dm_openarm::MitLoopController::safety_state)
    .def("fault", &dm_openarm::MitLoopController::fault)
    .def("accepted_sequence", &dm_openarm::MitLoopController::accepted_sequence)
    .def("sent_sequence", &dm_openarm::MitLoopController::sent_sequence)
    .def("stop", &dm_openarm::MitLoopController::stop, nb::call_guard<nb::gil_scoped_release>())
    .def("running", &dm_openarm::MitLoopController::running)
    .def("deadline_misses", &dm_openarm::MitLoopController::deadline_misses)
    .def("timing_stats", &dm_openarm::MitLoopController::timing_stats)
    .def(
      "set_command",
      &dm_openarm::MitLoopController::set_command,
      nb::arg("can_id"),
      nb::arg("command"))
    .def(
      "set_all_commands",
      &dm_openarm::MitLoopController::set_all_commands,
      nb::arg("commands"))
    .def("commands", &dm_openarm::MitLoopController::commands)
    .def("sent_commands", &dm_openarm::MitLoopController::sent_commands);
}
