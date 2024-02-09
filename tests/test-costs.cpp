#include "pinocchio/parsers/urdf.hpp"
 
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics-derivatives.hpp>
#include <pinocchio/algorithm/rnea-derivatives.hpp>
#include "qrw/ResidualFlyHigh.hpp"
#include "crocoddyl/core/numdiff/residual.hpp"
#include "crocoddyl/core/residuals/control.hpp"
#include "crocoddyl/multibody/data/multibody.hpp"
#include "crocoddyl/multibody/residuals/state.hpp"
#include "crocoddyl/multibody/actuations/floating-base.hpp"
#include <boost/function.hpp>


#include <iostream>

#include <boost/test/unit_test.hpp>

BOOST_AUTO_TEST_SUITE(costs)

void updateAllPinocchio(pinocchio::Model* const model, pinocchio::Data* data, const Eigen::VectorXd& x, const Eigen::VectorXd&) {
  const Eigen::VectorXd& q = x.segment(0, model->nq);
  const Eigen::VectorXd& v = x.segment(model->nq, model->nv);
  Eigen::VectorXd a = Eigen::VectorXd::Zero(model->nv);
//   Eigen::Matrix<double, 6, Eigen::Dynamic> tmp;
//   tmp.resize(6, model->nv);
  pinocchio::forwardKinematics(*model, *data, q, v, a);
  pinocchio::computeForwardKinematicsDerivatives(*model, *data, q, v, a);
  pinocchio::computeJointJacobians(*model, *data, q);
  pinocchio::updateFramePlacements(*model, *data);
//   pinocchio::centerOfMass(*model, *data, q, v, a);
//   pinocchio::jacobianCenterOfMass(*model, *data, q);
//   pinocchio::computeCentroidalMomentum(*model, *data, q, v);
//   pinocchio::computeCentroidalDynamicsDerivatives(*model, *data, q, v, a, tmp,
                                                //   tmp, tmp, tmp);
  pinocchio::computeRNEADerivatives(*model, *data, q, v, a);
}

void updateActuation(
    const boost::shared_ptr<crocoddyl::ActuationModelAbstract>& model,
    const boost::shared_ptr<crocoddyl::ActuationDataAbstract>& data,
    const Eigen::VectorXd& x, const Eigen::VectorXd& u) {
  model->calc(data, x, u);
}

BOOST_AUTO_TEST_CASE(test_partial_derivatives_against_numdiff) {
    // Pinocchio model and data
    const std::string urdf_filename = std::string(EXAMPLE_ROBOT_DATA_MODEL_DIR "/solo_description/robots/solo12.urdf");
    pinocchio::Model pin_model;
    pinocchio::urdf::buildModel(urdf_filename, pin_model);
    pinocchio::Data pin_data(pin_model);

    pinocchio::FrameIndex id = pin_model.getFrameId("FL_FOOT");

    // Crocoddyl state, actuation and residual models
    boost::shared_ptr<crocoddyl::StateMultibody> state  = boost::make_shared<crocoddyl::StateMultibody>(boost::make_shared<pinocchio::Model>(pin_model));
    boost::shared_ptr<crocoddyl::ActuationModelFloatingBase> actuation_model = boost::make_shared<crocoddyl::ActuationModelFloatingBase>(state);
    boost::shared_ptr<qrw::ResidualModelFlyHigh> flyhigh_cost_model = boost::make_shared<qrw::ResidualModelFlyHigh>(state, id, 0.1, actuation_model->get_nu());

    // create corresponding datas
    const boost::shared_ptr<crocoddyl::ActuationDataAbstract>& actuation_data = actuation_model->createData();
    crocoddyl::DataCollectorActMultibody shared_data(&pin_data, actuation_data);
    const boost::shared_ptr<crocoddyl::ResidualDataAbstract>& flyhigh_cost_data = flyhigh_cost_model->createData(&shared_data);

    // Generating random values for the state and control
    const Eigen::VectorXd x = state->rand();
    const Eigen::VectorXd u = Eigen::VectorXd::Random(actuation_model->get_nu());

    updateAllPinocchio(&pin_model, &pin_data,x, u);
    actuation_model->calc(actuation_data, x, u);

    // Create the equivalent num diff model and data.
    crocoddyl::ResidualModelNumDiff model_num_diff(flyhigh_cost_model);
    const boost::shared_ptr<crocoddyl::ResidualDataAbstract>& data_num_diff = model_num_diff.createData(&shared_data);

    // set the function that needs to be called at every step of the numdiff
    std::vector<crocoddyl::ResidualModelNumDiff::ReevaluationFunction> reevals;
    reevals.push_back(boost::bind(&updateAllPinocchio, &pin_model, &pin_data, boost::placeholders::_1, boost::placeholders::_2));
    reevals.push_back(boost::bind(&updateActuation, actuation_model, actuation_data, boost::placeholders::_1, boost::placeholders::_2));
    model_num_diff.set_reevals(reevals);

    // Computing the cost derivatives
    flyhigh_cost_model->calc(flyhigh_cost_data, x, u);
    flyhigh_cost_model->calcDiff(flyhigh_cost_data, x, u);
    model_num_diff.calc(data_num_diff, x, u);
    model_num_diff.calcDiff(data_num_diff, x, u);
    // Tolerance defined as in
    // http://www.it.uom.gr/teaching/linearalgebra/NumericalRecipiesInC/c5-7.pdf
    double tol = std::pow(model_num_diff.get_disturbance(), 1. / 3.);
    BOOST_CHECK((flyhigh_cost_data->Rx - data_num_diff->Rx).isZero(tol));
    BOOST_CHECK((flyhigh_cost_data->Ru - data_num_diff->Ru).isZero(tol));
}

BOOST_AUTO_TEST_SUITE_END()