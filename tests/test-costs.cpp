#include "pinocchio/parsers/urdf.hpp"
 
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics-derivatives.hpp>
#include <pinocchio/algorithm/rnea-derivatives.hpp>
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
    const Eigen::VectorXd& q = x.segment(0, pin_model.nq);
    const Eigen::VectorXd& v = x.segment(pin_model.nq, pin_model.nv);
    Eigen::VectorXd a = Eigen::VectorXd::Zero(pin_model.nv);

    pinocchio::forwardKinematics(pin_model, pin_data, q, v, a);
    pinocchio::computeForwardKinematicsDerivatives(pin_model, pin_data, q, v, a);
    pinocchio::computeJointJacobians(pin_model, pin_data, q);
    pinocchio::updateFramePlacements(pin_model, pin_data);
    pinocchio::computeRNEADerivatives(pin_model, pin_data, q, v, a);

    actuation_model->calc(actuation_data, x, u);

    std::cout << "x: "<< x << std::endl;
    std::cout << "u: "<< u << std::endl;

    // Getting the residual value computed by calc()
    data->r *= nan("");
    flyhigh_cost_model.calc(data, x, u);

    flyhigh_cost_model.calcDiff(data, x, u);
}

BOOST_AUTO_TEST_SUITE_END()