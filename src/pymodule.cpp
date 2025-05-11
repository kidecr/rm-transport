#ifdef USE_PYTHON
#include <pybind11/pybind11.h>
#include "external-interface/BluetoothInterface.hpp"
/**
 * @note 以下为对于Python的接口封装
 * 
 */
namespace py = pybind11;

PYBIND11_MODULE(transport_py, m) {
    // 注册PortException到Python
    py::register_exception<transport::PortException>(m, "PortException", PyExc_RuntimeError);
    
    py::class_<BluetoothInterface>(m, "_BluetoothInterface")
        .def(py::init<std::string>(), 
            py::arg("config_path"),
            "Constructs a BluetoothInterface with the given configuration path.")
        .def("_recvIMU", &BluetoothInterface::recvIMU,
            "Receive IMU data via Bluetooth and return as a dictionary.")
        .def("_getAvailablePortName", &BluetoothInterface::getAvailablePortName,
            "Get the available port name.")
        .def("_portIsAvailable", &BluetoothInterface::portIsAvailable,
            py::arg("port_name"),
            "Check if the port is available.");

}

#endif // USE_PYTHON