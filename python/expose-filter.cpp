#include "qrw/LowPassFilter.hpp"
#include "qrw/bindings/python.hpp"

namespace qrw {

template <typename Filter>
struct FilterVisitor : public bp::def_visitor<FilterVisitor<Filter>> {
  template <class PyClassFilter>
  void visit(PyClassFilter& cl) const {
    cl.def(bp::init<Params const&>(bp::args("self", "params"), "Default constructor."))
        .def("filter", &Filter::filter, bp::args("x", "check_modulo"), "Run Filter from Python.\n");
  }

  static void expose() { bp::class_<Filter>("LowPassFilter", bp::no_init).def(FilterVisitor<Filter>()); }
};

void exposeFilter() { FilterVisitor<LowPassFilter>::expose(); }

}  // namespace qrw
