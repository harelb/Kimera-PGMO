#pragma once

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace kimera_pgmo {

#if GTSAM_VERSION_MAJOR <= 4 && GTSAM_VERSION_MINOR < 3
using PositionPriorJacobianType = boost::optional<gtsam::Matrix&>;
#define JACOBIAN_DEFAULT \
  {}
#else
using PositionPriorJacobianType = gtsam::OptionalMatrixType;
#define JACOBIAN_DEFAULT nullptr
#endif

/*! \brief Absolute position measurement of a point rigidly attached to a pose node
 * (e.g. a GPS antenna on the robot body). The error is
 * world_T_body * body_t_sensor - world_t_sensor, so rotation is only constrained
 * through the lever arm.
 */
class PositionPriorFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
 private:
  gtsam::Point3 measurement_;
  gtsam::Point3 body_t_sensor_;

 public:
  PositionPriorFactor(gtsam::Key key,
                      const gtsam::Point3& world_t_sensor,
                      const gtsam::Point3& body_t_sensor,
                      gtsam::SharedNoiseModel model)
      : gtsam::NoiseModelFactor1<gtsam::Pose3>(model, key),
        measurement_(world_t_sensor),
        body_t_sensor_(body_t_sensor) {}

  virtual ~PositionPriorFactor() {}

  gtsam::Vector evaluateError(const gtsam::Pose3& pose,
                              PositionPriorJacobianType H = JACOBIAN_DEFAULT) const override {
    gtsam::Matrix36 H_pose;
    const gtsam::Point3 world_t_sensor = pose.transformFrom(body_t_sensor_, H_pose);
    if (H) {
      *H = H_pose;
    }

    return world_t_sensor - measurement_;
  }

  inline const gtsam::Point3& measurement() const { return measurement_; }

  inline const gtsam::Point3& bodyTSensor() const { return body_t_sensor_; }

  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return gtsam::NonlinearFactor::shared_ptr(new PositionPriorFactor(*this));
  }
};

#undef JACOBIAN_DEFAULT

}  // namespace kimera_pgmo
