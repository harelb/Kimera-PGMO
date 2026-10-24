/**
 * @file   test_position_prior_factor.cpp
 * @brief  Unit-tests for absolute position (GPS) priors
 */

#include <gtest/gtest.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <kimera_rpgo/rpgo.h>

#include "kimera_pgmo/deformation_graph.h"
#include "kimera_pgmo/position_prior_factor.h"

using gtsam::Point3;
using gtsam::Pose3;
using gtsam::Rot3;

namespace kimera_pgmo {
namespace {

gtsam::Values optimize(const DeformationGraph& graph) {
  kimera_rpgo::RpgoConfig pgo_config;
  pgo_config.solver_config.setLeastSquaresParamsDefault();
  kimera_rpgo::Rpgo pgo(pgo_config);
  pgo.addFactors(*graph.getFactors());
  pgo.addValues(*graph.getValues());
  pgo.run();
  return pgo.getResult();
}

void checkJacobian(const PositionPriorFactor& factor,
                   const Pose3& pose,
                   const gtsam::Vector& expected) {
  gtsam::Matrix H_actual;
#if GTSAM_VERSION_MAJOR <= 4 && GTSAM_VERSION_MINOR < 3
  const auto actual = factor.evaluateError(pose, H_actual);
#else
  const auto actual = factor.evaluateError(pose, &H_actual);
#endif
  const auto H_expected = gtsam::numericalDerivative11<gtsam::Vector, Pose3>(
      [&](const Pose3& p) { return factor.evaluateError(p); }, pose, 1.0e-5);
  EXPECT_TRUE(gtsam::assert_equal(expected, actual, 1.0e-6));
  EXPECT_TRUE(gtsam::assert_equal(H_expected, H_actual, 1.0e-5));
}

}  // namespace

TEST(TestPositionPriorFactor, noLeverArm) {
  const auto noise = gtsam::noiseModel::Isotropic::Variance(3, 1.0);
  PositionPriorFactor factor(1, Point3(1, 2, 3), Point3::Zero(), noise);
  const Pose3 pose(Rot3::Ypr(0.3, 0.1, -0.2), Point3(2, 2, 2));
  checkJacobian(factor, pose, (gtsam::Vector(3) << 1, 0, -1).finished());
}

TEST(TestPositionPriorFactor, leverArm) {
  const auto noise = gtsam::noiseModel::Isotropic::Variance(3, 1.0);
  const Point3 lever(-0.22, 0.0, 0.54);
  // yaw of 90 degrees maps body x onto world y
  const Pose3 pose(Rot3::Yaw(M_PI / 2.0), Point3(10, 20, 0));
  const Point3 expected_antenna(10.0, 20.0 - 0.22, 0.54);
  PositionPriorFactor factor(1, expected_antenna, lever, noise);
  checkJacobian(factor, pose, gtsam::Vector3::Zero());

  PositionPriorFactor offset(1, expected_antenna + Point3(1, 0, 0), lever, noise);
  checkJacobian(offset, Pose3(Rot3::Ypr(0.5, -0.1, 0.2), Point3(1, 2, 3)),
                offset.evaluateError(Pose3(Rot3::Ypr(0.5, -0.1, 0.2), Point3(1, 2, 3))));
}

TEST(TestDeformationGraph, positionPriorsAlignTrajectory) {
  // ground truth: robot drives along world x with a lever-armed antenna
  const Point3 lever(-0.22, 0.0, 0.54);
  std::vector<Pose3> truth;
  for (size_t i = 0; i < 5; ++i) {
    truth.emplace_back(Rot3::Yaw(0.1 * i), Point3(100.0 + 2.0 * i, 50.0 + 0.5 * i * i, 0));
  }

  // odometric frame is rotated and shifted relative to the world frame
  const Pose3 odom_T_world(Rot3::Yaw(1.2), Point3(-30, 7, 0.5));
  DeformationGraph graph;
  DeformationGraph::PositionMeasurements measurements;
  for (size_t i = 0; i < truth.size(); ++i) {
    const gtsam::Symbol key('a', i);
    graph.addNewNode(key, i, odom_T_world * truth[i], false);
    if (i > 0) {
      graph.processNewBetween(
          gtsam::Symbol('a', i - 1), key, truth[i - 1].between(truth[i]), 1.0e-6);
    }

    auto& meas = measurements.emplace_back();
    meas.key = key;
    meas.world_t_sensor = truth[i].transformFrom(lever);
    meas.covariance = Eigen::Matrix3d::Identity() * 1.0e-4;
    meas.body_t_sensor = lever;
  }

  // unknown node is skipped
  auto& bad = measurements.emplace_back();
  bad.key = gtsam::Symbol('b', 0);
  bad.world_t_sensor = Point3::Zero();
  bad.covariance = Eigen::Matrix3d::Identity();
  bad.known_inlier = true;

  EXPECT_EQ(truth.size(), graph.processPositionMeasurements(measurements));
  EXPECT_TRUE(graph.getKnownInlierSet()->empty());

  // the current estimate is not overwritten
  EXPECT_TRUE(gtsam::assert_equal(odom_T_world * truth[0],
                                  graph.getValues()->at<Pose3>(gtsam::Symbol('a', 0))));

  const auto result = optimize(graph);
  for (size_t i = 0; i < truth.size(); ++i) {
    EXPECT_TRUE(gtsam::assert_equal(truth[i], result.at<Pose3>(gtsam::Symbol('a', i)), 1.0e-3))
        << "node " << i;
  }
}

TEST(TestDeformationGraph, removePositionPriorsWithPrefix) {
  DeformationGraph graph;
  graph.addNewNode(gtsam::Symbol('a', 0), 0, Pose3(), false);
  graph.addNewNode(gtsam::Symbol('b', 0), 0, Pose3(), false);
  graph.processNewBetween(gtsam::Symbol('a', 0), gtsam::Symbol('b', 0), Pose3());

  DeformationGraph::PositionMeasurements measurements(2);
  measurements[0].key = gtsam::Symbol('a', 0);
  measurements[0].world_t_sensor = Point3(1, 0, 0);
  measurements[0].covariance = Eigen::Matrix3d::Identity();
  measurements[0].known_inlier = true;
  measurements[1].key = gtsam::Symbol('b', 0);
  measurements[1].world_t_sensor = Point3(1, 0, 0);
  measurements[1].covariance = Eigen::Matrix3d::Identity();
  measurements[1].known_inlier = true;

  const auto num_before = graph.getFactors()->size();
  EXPECT_EQ(2u, graph.processPositionMeasurements(measurements));
  ASSERT_EQ(num_before + 2, graph.getFactors()->size());
  EXPECT_EQ(2u, graph.getKnownInlierSet()->size());

  graph.removePositionPriorsWithPrefix('a');
  const auto factors = graph.getFactors();
  ASSERT_EQ(num_before + 1, factors->size());
  const auto remaining = dynamic_cast<const PositionPriorFactor*>(factors->back().get());
  ASSERT_TRUE(remaining != nullptr);
  EXPECT_EQ(gtsam::Symbol('b', 0).key(), remaining->key());

  // known inlier index follows the remaining factor
  const auto inliers = graph.getKnownInlierSet();
  ASSERT_EQ(1u, inliers->size());
  EXPECT_EQ(factors->size() - 1, *inliers->begin());
}

}  // namespace kimera_pgmo
