#ifndef __PYTHON_ADDER__
#define __PYTHON_ADDER__

#include <eigenpy/eigenpy.hpp>

namespace bp = boost::python;

void exposeAnimators();
void exposeParams();
void exposeEstimator();
void exposeFilter();

#endif
