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
    .def_ro("feedback_interval_s", &dm_openarm::MotorState::feedback_interval_s)
    .def_ro("last_rx_age_s", &dm_openarm::MotorState::last_rx_age_s)
    .def_ro("rx_sequence", &dm_openarm::MotorState::rx_sequence)
    .def_ro("error_code", &dm_openarm::MotorState::error_code)
    .def("feedback_fresh", &dm_openarm::MotorState::feedback_fresh,
         nb::arg("max_age_s") = 0.1);

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
    .def("enable", &dm_openarm::DmArm::enable)
    .def("disable", &dm_openarm::DmArm::disable)
    .def("disconnect", &dm_openarm::DmArm::disconnect)
    .def("connected", &dm_openarm::DmArm::connected)
    .def("send_mit_all", &dm_openarm::DmArm::send_mit_all, nb::arg("commands"))
    .def("send_zero_mit_all", &dm_openarm::DmArm::send_zero_mit_all)
    .def(
      "set_zero",
      &dm_openarm::DmArm::set_zero,
      nb::arg("can_id"),
      nb::arg("persist") = true)
    .def("set_zero_all", &dm_openarm::DmArm::set_zero_all, nb::arg("persist") = true)
    .def("states", &dm_openarm::DmArm::states);

  nb::class_<dm_openarm::MitLoopController>(m, "MitLoopController")
    .def(nb::init<dm_openarm::DmArm&>(), nb::keep_alive<1, 2>())
    .def("start", &dm_openarm::MitLoopController::start,
         nb::arg("hz") = dm_openarm::kDefaultControlHz)
    .def("stop", &dm_openarm::MitLoopController::stop)
    .def("running", &dm_openarm::MitLoopController::running)
    .def("deadline_misses", &dm_openarm::MitLoopController::deadline_misses)
    .def(
      "set_command",
      &dm_openarm::MitLoopController::set_command,
      nb::arg("can_id"),
      nb::arg("command"))
    .def(
      "set_all_commands",
      &dm_openarm::MitLoopController::set_all_commands,
      nb::arg("commands"))
    .def("commands", &dm_openarm::MitLoopController::commands);
}
