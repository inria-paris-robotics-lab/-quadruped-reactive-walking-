#include "qrw/Filter.hpp"
#include "qrw/bindings/python.hpp"

namespace qrw {

template <typename Filter>
struct FilterVisitor : public bp::def_visitor<FilterVisitor<Filter>> {
  template <class PyClassFilter>
  void visit(PyClassFilter& cl) const {
    cl.def(bp::init<>(bp::args("self"), "Default constructor."))
        .def("initialize", &Filter::initialize, bp::args("params"),
             "Initialize Filter from Python.\n")
        .def("filter", &Filter::filter, bp::args("x", "check_modulo"),
             "Run Filter from Python.\n");
  }

  static void expose() {
    bp::class_<Filter>("Filter", bp::no_init).def(FilterVisitor<Filter>());
  }
};

void exposeFilter() { FilterVisitor<Filter>::expose(); }

}  // namespace qrw
