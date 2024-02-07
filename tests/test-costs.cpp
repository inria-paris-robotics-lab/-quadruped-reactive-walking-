#include "pinocchio/parsers/urdf.hpp"
 
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "qrw/ResidualFlyHigh.hpp"
#include "crocoddyl/core/residuals/control.hpp"
#include "crocoddyl/multibody/data/multibody.hpp"
#include "crocoddyl/multibody/residuals/state.hpp"
#include "crocoddyl/multibody/actuations/floating-base.hpp"

#include <iostream>

#include <boost/test/unit_test.hpp>

BOOST_AUTO_TEST_SUITE(costs)

BOOST_AUTO_TEST_CASE(test_partial_derivatives_against_numdiff) {
    // Pinocchio model and data
    const std::string urdf_filename = std::string(EXAMPLE_ROBOT_DATA_MODEL_DIR "/solo_description/robots/solo12.urdf");
    pinocchio::Model pin_model;
    pinocchio::urdf::buildModel(urdf_filename, pin_model);
    pinocchio::Data pin_data(pin_model);

    pinocchio::FrameIndex id = pin_model.getFrameId("FL_FOOT");

    // Crocoddyl state and cost
    boost::shared_ptr<crocoddyl::StateMultibody> state  = boost::make_shared<crocoddyl::StateMultibody>((boost::make_shared<pinocchio::Model>(pin_model)));
    qrw::ResidualModelFlyHigh flyhigh_cost_model(state, id, 0.1, state->get_nv());

    // create the residual data
    boost::shared_ptr<crocoddyl::ActuationModelFloatingBase> actuation_model = boost::make_shared<crocoddyl::ActuationModelFloatingBase>(state);
    const boost::shared_ptr<crocoddyl::ActuationDataAbstract>& actuation_data = actuation_model->createData();
    crocoddyl::DataCollectorActMultibody shared_data(&pin_data, actuation_data);

    const boost::shared_ptr<crocoddyl::ResidualDataAbstract>& data = flyhigh_cost_model.createData(&shared_data);

    // Generating random values for the state and control
    const Eigen::VectorXd x = state->rand();
    const Eigen::VectorXd u = Eigen::VectorXd::Random(actuation_model->get_nu());

    // // Compute all the pinocchio function needed for the models.
    // crocoddyl::unittest::updateAllPinocchio(&pin_model, &pin_data, x);
    // crocoddyl::unittest::updateActuation(actuation_model, actuation_data, x, u);

    // Getting the residual value computed by calc()
    data->r *= nan("");
    flyhigh_cost_model.calc(data, x, u);
}

BOOST_AUTO_TEST_SUITE_END()