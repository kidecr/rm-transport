#include <pybind11/pybind11.h>
#include "external-interface/BluetoothInterface.hpp"

/**
 * @note 以下为对于Python的接口封装
 * 
 */
namespace py = pybind11;

PYBIND11_MODULE(transport_py, m) {
    py::class_<BluetoothInterface>(m, "_BluetoothInterface")
        .def(py::init<std::string>(), 
            py::arg("config_path"),
            "Constructs a BluetoothInterface with the given configuration path.")
        .def("_recvIMU", &BluetoothInterface::recvIMU,
            "Receive IMU data via Bluetooth and return as a dictionary.")
        .def("_getAvaliablePortName", &BluetoothInterface::getAvailablePortName,
            "Get the available port name.");

}