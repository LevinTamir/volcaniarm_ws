// Python bindings for volcaniarm_kinematics so ROS Python nodes (teleop,
// calibration, weed pursuit) use the single C++ source of truth instead of a
// duplicate IK service. Import as: `import volcaniarm_kinematics_py as vk`.
#include <pybind11/pybind11.h>

#include "volcaniarm_kinematics/five_bar.hpp"
#include "volcaniarm_kinematics/params.hpp"

namespace py = pybind11;
using namespace volcaniarm_kinematics;

PYBIND11_MODULE(volcaniarm_kinematics_py, m)
{
  m.doc() = "Volcaniarm planar five-bar kinematics (C++ single source of truth)";

  py::class_<Params>(m, "Params")
    .def(py::init<>())
    .def_readwrite("L1", &Params::L1)
    .def_readwrite("L2", &Params::L2)
    .def_readwrite("l0", &Params::l0)
    .def_readwrite("base_z", &Params::base_z)
    .def_readwrite("arm_lateral", &Params::arm_lateral)
    .def_readwrite("left_elbow_rpy", &Params::left_elbow_rpy)
    .def_readwrite("right_elbow_rpy", &Params::right_elbow_rpy)
    .def_readwrite("left_arm_rpy", &Params::left_arm_rpy)
    .def_readwrite("right_arm_rpy", &Params::right_arm_rpy)
    .def_readwrite("closure_rpy", &Params::closure_rpy);

  py::class_<Point2>(m, "Point2")
    .def_readonly("y", &Point2::y)
    .def_readonly("z", &Point2::z);

  py::class_<EEResult>(m, "EEResult")
    .def_readonly("y", &EEResult::y)
    .def_readonly("z", &EEResult::z)
    .def_readonly("valid", &EEResult::valid);

  py::class_<PassiveAngles>(m, "PassiveAngles")
    .def_readonly("left_arm", &PassiveAngles::left_arm)
    .def_readonly("right_arm", &PassiveAngles::right_arm)
    .def_readonly("closure", &PassiveAngles::closure)
    .def_readonly("valid", &PassiveAngles::valid);

  py::class_<EEPose>(m, "EEPose")
    .def_readonly("x", &EEPose::x)
    .def_readonly("y", &EEPose::y)
    .def_readonly("z", &EEPose::z)
    .def_readonly("valid", &EEPose::valid);

  py::class_<ElbowAngles>(m, "ElbowAngles")
    .def_readonly("theta_left", &ElbowAngles::theta_left)
    .def_readonly("theta_right", &ElbowAngles::theta_right)
    .def_readonly("valid", &ElbowAngles::valid);

  m.def("elbow_tip", &elbowTip, py::arg("p"), py::arg("elbow_rpy"), py::arg("theta"),
    py::arg("shoulder_y"), py::arg("lateral"));
  m.def("circle_intersect", &circleIntersect, py::arg("p"), py::arg("y_l"), py::arg("z_l"),
    py::arg("y_r"), py::arg("z_r"));
  m.def("passive_angles", &passiveAngles, py::arg("p"), py::arg("theta_left"),
    py::arg("theta_right"));
  m.def("forward_ee", &forwardEE, py::arg("p"), py::arg("theta_left"), py::arg("theta_right"),
    py::arg("tool_offset") = 0.0);
  m.def("side_ik", &sideIK, py::arg("p"), py::arg("elbow_rpy"), py::arg("shoulder_y"),
    py::arg("lateral"), py::arg("y_tip"), py::arg("z_tip"), py::arg("theta_seed"));
  m.def("closure_margin", &closureMargin, py::arg("p"), py::arg("theta_L"), py::arg("theta_R"));
  m.def("inverse_ee", &inverseEE, py::arg("p"), py::arg("ee_y"), py::arg("ee_z"),
    py::arg("seed_left"), py::arg("seed_right"));
}
