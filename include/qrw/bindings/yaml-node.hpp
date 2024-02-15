#pragma once

#include <yaml-cpp/yaml.h>
#include <eigenpy/eigenpy.hpp>
#include <boost/python/module.hpp>

namespace qrw {

namespace bp = boost::python;

struct YamlNodeToPython {
  static PyObject *convert(const YAML::Node &obj) {
    YAML::Emitter emitter;
    emitter << obj;
    bp::str string(emitter.c_str());
    bp::object pyyaml = bp::import("yaml");
    bp::object loader = pyyaml.attr("full_load");
    bp::object d = loader(string);
    return bp::incref(d.ptr());
  }

  static void registration() { bp::to_python_converter<YAML::Node, YamlNodeToPython>(); }
};

}  // namespace qrw
