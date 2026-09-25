#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "scots.hh"

#include <array>
#include <vector>
#include <string>

namespace py = pybind11;

using state_type = std::array<double,3>;
using input_type = std::array<double,3>;

class Controller {
    public:
    Controller(const std::string &filename) {
        if (!scots::read_from_file(con, filename)) {
        throw std::runtime_error("Could not read controller file");
        }
    }

    std::vector<input_type> get_control(const state_type &x) const {
        return con.peek_control<state_type, input_type>(x);
    }

    private:
    scots::StaticController con;
};

PYBIND11_MODULE(scots_ctrl, m) {
  py::class_<Controller>(m, "Controller")
      .def(py::init<const std::string &>())
      .def("get_control", &Controller::get_control);
}