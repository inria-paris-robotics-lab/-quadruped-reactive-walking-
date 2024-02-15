#pragma once

#include <eigenpy/fwd.hpp>
#include <vector>

namespace qrw {

namespace {
namespace bp = boost::python;
}

namespace detail {

/// Util class to raise error when stl container member is resized.
template <typename Class, typename vector_type>
struct non_resizable_vec_member {
 private:
  vector_type Class::*m_which;

 public:
  non_resizable_vec_member(vector_type Class::*which) : m_which(which) {}

  void operator()(Class &c, const vector_type &rhs) {
    auto &lhs = c.*m_which;
    if (lhs.size() != rhs.size()) {
      std::ostringstream ss;
      ss << "vector size is non-mutable (";
      ss << "expected " << lhs.size();
      ss << ", got " << rhs.size() << ").";
      PyErr_SetString(PyExc_ValueError, ss.str().c_str());
      bp::throw_error_already_set();
    }
    lhs = rhs;
  }
};
}  // namespace detail

template <typename Class, typename vector_type>
bp::object make_non_resizable_vec_member(vector_type Class::*which) {
  return bp::make_function(detail::non_resizable_vec_member<Class, vector_type>(which),
                           bp::default_call_policies(),
                           boost::mpl::vector3<void, Class &, const vector_type &>());
}

}  // namespace qrw
